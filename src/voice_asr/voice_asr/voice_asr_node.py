#!/usr/bin/env python3

"""
三版本 ROS2 语音识别节点。

入口1：voice_asr_raw_internal
    内置麦原始 /aima/hal/audio/capture
    -> 6总声道 / 4麦克风 + 2参考
    -> mic channel 2 / gain 14.0
    -> 本地 Silero VAD
    -> SenseVoice

入口2：voice_asr_raw_external
    外置麦原始 /aima/hal/audio/capture
    -> 2总声道 / 1麦克风 + 1参考
    -> mic channel 0 / gain 1.0
    -> 本地 Silero VAD
    -> SenseVoice

入口3：voice_asr_processed_external
    外置麦 /agent/process_audio_output
    -> AimDK 上游 BEGIN/PROCESSING/END 分段
    -> 不使用本地 Silero VAD
    -> SenseVoice

三个版本均保持原有业务接口：
    订阅 /ai_agent/listen_control (Bool)
    发布 /ai_agent/recognized_text (String)
"""

from __future__ import annotations

import queue
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from .asr_engine import SenseVoiceEngine
from .external_mic import (
    ensure_external_mic,
    ensure_internal_mic,
)
from .listening_session import (
    ListeningSession,
    ListeningSessionResult,
    ListeningStatus,
)
from .processed_audio_source import ProcessedAudioSource
from .processed_listening_session import ProcessedListeningSession
from .robot_audio_source import RobotAudioSource
from .vad_engine import SileroVadEngine


MODE_RAW_INTERNAL = "raw_internal"
MODE_RAW_EXTERNAL = "raw_external"
MODE_PROCESSED_EXTERNAL = "processed_external"

DEFAULT_LISTEN_CONTROL_TOPIC = "/ai_agent/listen_control"
DEFAULT_RECOGNIZED_TEXT_TOPIC = "/ai_agent/recognized_text"
DEFAULT_RAW_AUDIO_TOPIC = "/aima/hal/audio/capture"
DEFAULT_PROCESSED_AUDIO_TOPIC = "/agent/process_audio_output"
DEFAULT_SET_MIC_SERVICE = "/aimdk_5Fmsgs/srv/SetMicSourceRequest"
DEFAULT_GET_MIC_SERVICE = "/aimdk_5Fmsgs/srv/GetMicSourceRequest"


class VoiceAsrNode(Node):
    """执行单轮监听与 SenseVoice 识别。"""

    def __init__(self, *, mode: str) -> None:
        if mode not in (
            MODE_RAW_INTERNAL,
            MODE_RAW_EXTERNAL,
            MODE_PROCESSED_EXTERNAL,
        ):
            raise ValueError(f"不支持的ASR模式：{mode}")

        super().__init__("voice_asr_node")
        self._mode = mode

        self._declare_parameters()
        self._read_parameters()

        self._worker_lock = threading.Lock()
        self._worker_thread: threading.Thread | None = None
        self._round_sequence = 0
        self._active_round_id: int | None = None
        self._shutting_down = False
        self._recognized_queue: queue.Queue[str] = queue.Queue()

        self._raw_audio_subscription = None
        self._processed_audio_subscription = None
        self._vad_engine: SileroVadEngine | None = None

        if self._force_mic_source:
            if self._mode == MODE_RAW_INTERNAL:
                self.get_logger().info(
                    "正在切换并确认内置麦克风……"
                )
                actual_source = ensure_internal_mic(
                    self,
                    set_service_name=self._set_mic_source_service,
                    get_service_name=self._get_mic_source_service,
                    timeout_sec=self._mic_service_timeout,
                )
                self.get_logger().info(
                    "内置麦克风已确认："
                    f"mic_source={actual_source}"
                )
            else:
                self.get_logger().info(
                    "正在切换并确认外置麦克风……"
                )
                actual_source = ensure_external_mic(
                    self,
                    set_service_name=self._set_mic_source_service,
                    get_service_name=self._get_mic_source_service,
                    timeout_sec=self._mic_service_timeout,
                    external_source_id=1,
                )
                self.get_logger().info(
                    "外置麦克风已确认："
                    f"mic_source={actual_source}"
                )

        self.get_logger().info("正在初始化语音识别引擎……")
        self._create_engines()

        self._recognized_text_publisher = self.create_publisher(
            String,
            self._recognized_text_topic,
            10,
        )

        self._listen_control_subscription = self.create_subscription(
            Bool,
            self._listen_control_topic,
            self._on_listen_control,
            10,
        )

        self._publish_timer = self.create_timer(
            0.05,
            self._publish_queued_results,
        )

        self._log_startup_summary()

    def _declare_parameters(self) -> None:
        home = Path.home()
        models_dir = home / "star_agibot" / "models"

        self.declare_parameter(
            "asr_model_path",
            str(models_dir / "sensevoice_int8" / "model.int8.onnx"),
        )
        self.declare_parameter(
            "tokens_path",
            str(models_dir / "sensevoice_int8" / "tokens.txt"),
        )
        self.declare_parameter(
            "vad_model_path",
            str(models_dir / "vad" / "silero_vad.onnx"),
        )

        self.declare_parameter(
            "listen_control_topic",
            DEFAULT_LISTEN_CONTROL_TOPIC,
        )
        self.declare_parameter(
            "recognized_text_topic",
            DEFAULT_RECOGNIZED_TEXT_TOPIC,
        )

        self.declare_parameter("force_mic_source", True)
        self.declare_parameter(
            "set_mic_source_service",
            DEFAULT_SET_MIC_SERVICE,
        )
        self.declare_parameter(
            "get_mic_source_service",
            DEFAULT_GET_MIC_SERVICE,
        )
        self.declare_parameter("mic_service_timeout", 5.0)

        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("num_threads", 2)
        self.declare_parameter("max_wait_seconds", 20.0)
        self.declare_parameter("language", "zh")
        self.declare_parameter("use_itn", False)

        # 两个原始音频版本共用这些参数；
        # 内/外置麦的具体值由各自 YAML 指定。
        self.declare_parameter(
            "raw_audio_topic",
            DEFAULT_RAW_AUDIO_TOPIC,
        )
        self.declare_parameter("raw_audio_channel", 0)
        self.declare_parameter("raw_audio_gain", 1.0)
        self.declare_parameter("raw_chunk_samples", 1600)
        self.declare_parameter("raw_expected_total_channels", 2)
        self.declare_parameter("raw_expected_mic_channels", 1)
        self.declare_parameter("raw_expected_ref_channels", 1)

        self.declare_parameter("vad_threshold", 0.2)
        self.declare_parameter("vad_min_silence_duration", 0.8)
        self.declare_parameter("vad_min_speech_duration", 0.25)
        self.declare_parameter("vad_max_speech_duration", 10.0)

        # 处理后外置麦版本参数。
        self.declare_parameter(
            "processed_audio_topic",
            DEFAULT_PROCESSED_AUDIO_TOPIC,
        )
        self.declare_parameter(
            "processed_min_speech_duration",
            0.20,
        )
        self.declare_parameter(
            "processed_max_speech_duration",
            12.0,
        )
        self.declare_parameter(
            "processed_repeated_begin_gap",
            0.35,
        )

    def _read_parameters(self) -> None:
        self._asr_model_path = Path(
            str(self.get_parameter("asr_model_path").value)
        ).expanduser().resolve()
        self._tokens_path = Path(
            str(self.get_parameter("tokens_path").value)
        ).expanduser().resolve()
        self._vad_model_path = Path(
            str(self.get_parameter("vad_model_path").value)
        ).expanduser().resolve()

        self._listen_control_topic = str(
            self.get_parameter("listen_control_topic").value
        )
        self._recognized_text_topic = str(
            self.get_parameter("recognized_text_topic").value
        )

        self._force_mic_source = bool(
            self.get_parameter("force_mic_source").value
        )
        self._set_mic_source_service = str(
            self.get_parameter("set_mic_source_service").value
        )
        self._get_mic_source_service = str(
            self.get_parameter("get_mic_source_service").value
        )
        self._mic_service_timeout = float(
            self.get_parameter("mic_service_timeout").value
        )

        self._sample_rate = int(
            self.get_parameter("sample_rate").value
        )
        self._num_threads = int(
            self.get_parameter("num_threads").value
        )
        self._max_wait_seconds = float(
            self.get_parameter("max_wait_seconds").value
        )
        self._language = str(
            self.get_parameter("language").value
        )
        self._use_itn = bool(
            self.get_parameter("use_itn").value
        )

        self._raw_audio_topic = str(
            self.get_parameter("raw_audio_topic").value
        )
        self._raw_audio_channel = int(
            self.get_parameter("raw_audio_channel").value
        )
        self._raw_audio_gain = float(
            self.get_parameter("raw_audio_gain").value
        )
        self._raw_chunk_samples = int(
            self.get_parameter("raw_chunk_samples").value
        )
        self._raw_expected_total_channels = int(
            self.get_parameter("raw_expected_total_channels").value
        )
        self._raw_expected_mic_channels = int(
            self.get_parameter("raw_expected_mic_channels").value
        )
        self._raw_expected_ref_channels = int(
            self.get_parameter("raw_expected_ref_channels").value
        )

        self._vad_threshold = float(
            self.get_parameter("vad_threshold").value
        )
        self._vad_min_silence_duration = float(
            self.get_parameter("vad_min_silence_duration").value
        )
        self._vad_min_speech_duration = float(
            self.get_parameter("vad_min_speech_duration").value
        )
        self._vad_max_speech_duration = float(
            self.get_parameter("vad_max_speech_duration").value
        )

        self._processed_audio_topic = str(
            self.get_parameter("processed_audio_topic").value
        )
        self._processed_min_speech_duration = float(
            self.get_parameter("processed_min_speech_duration").value
        )
        self._processed_max_speech_duration = float(
            self.get_parameter("processed_max_speech_duration").value
        )
        self._processed_repeated_begin_gap = float(
            self.get_parameter("processed_repeated_begin_gap").value
        )

        self._validate_parameters()

    def _validate_parameters(self) -> None:
        if not self._listen_control_topic:
            raise ValueError("listen_control_topic不能为空")
        if not self._recognized_text_topic:
            raise ValueError("recognized_text_topic不能为空")
        if self._sample_rate != 16000:
            raise ValueError("当前两条语音链路固定使用16000 Hz")
        if self._num_threads < 1:
            raise ValueError("num_threads必须大于等于1")
        if self._max_wait_seconds <= 0:
            raise ValueError("max_wait_seconds必须大于0")
        if self._mic_service_timeout <= 0:
            raise ValueError("mic_service_timeout必须大于0")

    def _create_engines(self) -> None:
        self._asr_engine = SenseVoiceEngine(
            model_path=self._asr_model_path,
            tokens_path=self._tokens_path,
            sample_rate=self._sample_rate,
            num_threads=self._num_threads,
            language=self._language,
            use_itn=self._use_itn,
            provider="cpu",
            debug=False,
        )

        if self._mode in (
            MODE_RAW_INTERNAL,
            MODE_RAW_EXTERNAL,
        ):
            self._create_raw_pipeline()
        else:
            self._create_processed_external_pipeline()

        self.get_logger().info(
            "SenseVoice模型加载完成，"
            f"用时 {self._asr_engine.load_time:.3f} 秒。"
        )

    def _create_raw_pipeline(self) -> None:
        try:
            from aimdk_msgs.msg import AudioCapture
        except ImportError as exc:
            raise RuntimeError(
                "无法导入aimdk_msgs/msg/AudioCapture"
            ) from exc

        self._audio_source = RobotAudioSource(
            sample_rate=self._sample_rate,
            chunk_samples=self._raw_chunk_samples,
            selected_channel=self._raw_audio_channel,
            digital_gain=self._raw_audio_gain,
            expected_total_channels=self._raw_expected_total_channels,
            expected_mic_channels=self._raw_expected_mic_channels,
            expected_ref_channels=self._raw_expected_ref_channels,
        )

        self._vad_engine = SileroVadEngine(
            model_path=self._vad_model_path,
            sample_rate=self._sample_rate,
            threshold=self._vad_threshold,
            min_silence_duration=self._vad_min_silence_duration,
            min_speech_duration=self._vad_min_speech_duration,
            max_speech_duration=self._vad_max_speech_duration,
        )

        self._session = ListeningSession(
            audio_source=self._audio_source,
            vad_engine=self._vad_engine,
            asr_engine=self._asr_engine,
            max_wait_seconds=self._max_wait_seconds,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._raw_audio_subscription = self.create_subscription(
            AudioCapture,
            self._raw_audio_topic,
            self._on_raw_audio,
            qos,
        )

    def _create_processed_external_pipeline(self) -> None:
        try:
            from aimdk_msgs.msg import ProcessedAudioOutput
        except ImportError as exc:
            raise RuntimeError(
                "无法导入aimdk_msgs/msg/ProcessedAudioOutput"
            ) from exc

        self._audio_source = ProcessedAudioSource(
            sample_rate=self._sample_rate,
            min_speech_duration=(
                self._processed_min_speech_duration
            ),
            max_speech_duration=(
                self._processed_max_speech_duration
            ),
            repeated_begin_gap_seconds=(
                self._processed_repeated_begin_gap
            ),
        )

        self._session = ProcessedListeningSession(
            audio_source=self._audio_source,
            asr_engine=self._asr_engine,
            max_wait_seconds=self._max_wait_seconds,
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._processed_audio_subscription = self.create_subscription(
            ProcessedAudioOutput,
            self._processed_audio_topic,
            self._on_processed_audio,
            qos,
        )

    def _on_raw_audio(self, message) -> None:
        if self._mode in (
            MODE_RAW_INTERNAL,
            MODE_RAW_EXTERNAL,
        ):
            self._audio_source.accept_message(message)

    def _on_processed_audio(self, message) -> None:
        if self._mode == MODE_PROCESSED_EXTERNAL:
            self._audio_source.accept_message(message)

    def _on_listen_control(self, message: Bool) -> None:
        if message.data:
            self._start_listening_round()
        else:
            self._cancel_listening_round()

    def _start_listening_round(self) -> None:
        with self._worker_lock:
            if self._shutting_down:
                return

            if (
                self._worker_thread is not None
                and self._worker_thread.is_alive()
            ):
                self.get_logger().warning(
                    "当前已经在监听或识别，"
                    "忽略重复的 listen_control=true。"
                )
                return

            self._round_sequence += 1
            round_id = self._round_sequence
            self._active_round_id = round_id

            worker = threading.Thread(
                target=self._run_listening_round,
                args=(round_id,),
                name=f"voice_asr_round_{round_id}",
                daemon=True,
            )
            self._worker_thread = worker
            worker.start()

        self.get_logger().info(
            f"第 {round_id} 轮监听已开启，请开始说话。"
        )

    def _cancel_listening_round(self) -> None:
        with self._worker_lock:
            worker = self._worker_thread

            if worker is None or not worker.is_alive():
                self._active_round_id = None
                self.get_logger().info(
                    "收到 listen_control=false，"
                    "但当前未处于监听状态。"
                )
                return

            self._active_round_id = None

        self.get_logger().info(
            "收到 listen_control=false，正在取消当前监听。"
        )
        self._session.cancel()

    def _run_listening_round(self, round_id: int) -> None:
        result: ListeningSessionResult | None = None
        is_current_round = False

        try:
            result = self._session.run_once()
        except Exception as exc:
            self.get_logger().error(
                f"第 {round_id} 轮监听异常：{exc}"
            )
        finally:
            with self._worker_lock:
                is_current_round = (
                    self._active_round_id == round_id
                )

                if is_current_round:
                    self._active_round_id = None

                self._worker_thread = None

        if result is None:
            return

        if not is_current_round:
            self.get_logger().info(
                f"第 {round_id} 轮结果已失效，"
                "不发布识别文本。"
            )
            return

        self._handle_session_result(
            round_id=round_id,
            result=result,
        )

    def _handle_session_result(
        self,
        *,
        round_id: int,
        result: ListeningSessionResult,
    ) -> None:
        if result.status == ListeningStatus.RECOGNIZED:
            text = result.text.strip()
            if not text:
                self.get_logger().warning(
                    f"第 {round_id} 轮识别结果为空。"
                )
                return

            self._recognized_queue.put(text)

            decode_time = (
                result.asr_result.decode_time
                if result.asr_result is not None
                else 0.0
            )
            rtf = (
                result.asr_result.rtf
                if result.asr_result is not None
                else 0.0
            )

            self.get_logger().info(
                f"第 {round_id} 轮识别完成：{text}"
            )
            self.get_logger().info(
                f"ASR用时 {decode_time:.3f} 秒，"
                f"RTF {rtf:.3f}。"
            )
            return

        if result.status == ListeningStatus.TIMEOUT:
            self.get_logger().warning(
                f"第 {round_id} 轮监听超时，"
                "未获得有效语音。"
            )
            return

        if result.status == ListeningStatus.CANCELLED:
            self.get_logger().info(
                f"第 {round_id} 轮监听已取消。"
            )
            return

        if result.status == ListeningStatus.EMPTY_RESULT:
            self.get_logger().warning(
                f"第 {round_id} 轮获得语音，"
                "但ASR没有输出文字。"
            )
            return

        self.get_logger().warning(
            f"第 {round_id} 轮出现未知状态：{result.status}"
        )

    def _publish_queued_results(self) -> None:
        if self._shutting_down:
            return

        while True:
            try:
                text = self._recognized_queue.get_nowait()
            except queue.Empty:
                break

            message = String()
            message.data = text
            self._recognized_text_publisher.publish(message)
            self.get_logger().info(
                f"已发布 {self._recognized_text_topic}：{text}"
            )

    def _log_startup_summary(self) -> None:
        self.get_logger().info("voice_asr_node 初始化完成。")
        self.get_logger().info(f"运行模式：{self._mode}")

        if self._mode == MODE_RAW_INTERNAL:
            self.get_logger().info(
                "麦克风源：内置麦 mic_source=0"
            )
        else:
            self.get_logger().info(
                "麦克风源：外置麦 mic_source=1"
            )

        self.get_logger().info(
            f"监听控制话题：{self._listen_control_topic}"
        )
        self.get_logger().info(
            f"识别文本话题：{self._recognized_text_topic}"
        )

        if self._mode == MODE_RAW_INTERNAL:
            self.get_logger().info(
                f"原始音频话题：{self._raw_audio_topic}"
            )
            self.get_logger().info(
                "内置麦原始格式："
                "6总声道 / 4麦克风 / 2参考；"
                f"使用mic ch{self._raw_audio_channel}；"
                f"数字增益={self._raw_audio_gain}"
            )
            self.get_logger().info(
                "分段方式：本地 Silero VAD"
            )

        elif self._mode == MODE_RAW_EXTERNAL:
            self.get_logger().info(
                f"原始音频话题：{self._raw_audio_topic}"
            )
            self.get_logger().info(
                "外置麦原始格式："
                "2总声道 / 1麦克风 / 1参考；"
                f"使用mic ch{self._raw_audio_channel}；"
                f"数字增益={self._raw_audio_gain}"
            )
            self.get_logger().info(
                "分段方式：本地 Silero VAD"
            )

        else:
            self.get_logger().info(
                f"处理后音频话题：{self._processed_audio_topic}"
            )
            self.get_logger().info(
                "分段方式：AimDK audio_vad_state "
                "BEGIN/PROCESSING/END"
            )
            self.get_logger().info(
                "本模式不创建、不运行 Silero VAD。"
            )

        self.get_logger().info(
            "当前处于关闭状态，等待 listen_control=true。"
        )

    def destroy_node(self) -> bool:
        self._shutting_down = True

        with self._worker_lock:
            self._active_round_id = None
            worker = self._worker_thread

        self._session.cancel()

        if (
            worker is not None
            and worker.is_alive()
            and worker is not threading.current_thread()
        ):
            worker.join(timeout=2.0)

        self._audio_source.stop()

        if self._vad_engine is not None:
            self._vad_engine.reset()

        return super().destroy_node()


def _run(mode: str, args=None) -> None:
    rclpy.init(args=args)
    node: VoiceAsrNode | None = None

    try:
        node = VoiceAsrNode(mode=mode)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def main_raw_internal(args=None) -> None:
    _run(MODE_RAW_INTERNAL, args=args)


def main_raw_external(args=None) -> None:
    _run(MODE_RAW_EXTERNAL, args=args)


def main_processed_external(args=None) -> None:
    _run(MODE_PROCESSED_EXTERNAL, args=args)


def main(args=None) -> None:
    """向后兼容旧入口；默认恢复最初的内置麦原始音频版本。"""
    main_raw_internal(args=args)


if __name__ == "__main__":
    main()
