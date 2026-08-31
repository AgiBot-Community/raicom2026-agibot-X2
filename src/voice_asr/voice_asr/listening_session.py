#!/usr/bin/env python3

"""
单轮语音监听会话。

完整流程：

实时麦克风音频
→ Silero VAD检测一句话结束
→ 停止麦克风
→ SenseVoice识别
→ 返回结构化结果

本模块实现“一次开启，只识别一句话”的控制逻辑。

本模块不负责：
- ROS2话题订阅与发布；
- 意图识别；
- 任务状态机；
- 机器人动作控制。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from threading import Event, Lock
from time import monotonic

from .asr_engine import AsrResult, SenseVoiceEngine
from .vad_engine import SileroVadEngine, VadSegment


class ListeningStatus(str, Enum):
    """一轮监听可能产生的结果状态。"""

    RECOGNIZED = "recognized"
    EMPTY_RESULT = "empty_result"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class ListeningSessionResult:
    """一轮监听的结构化结果。"""

    status: ListeningStatus
    text: str
    elapsed_time: float
    vad_segment: VadSegment | None = None
    asr_result: AsrResult | None = None

    @property
    def recognized(self) -> bool:
        """本轮是否成功获得非空识别文本。"""
        return (
            self.status == ListeningStatus.RECOGNIZED
            and bool(self.text)
        )


class ListeningSession:
    """
    管理一次完整的实时语音监听和识别。

    一个ListeningSession对象可以重复使用，但同一时刻
    只能运行一轮监听。
    """

    def __init__(
        self,
        *,
        audio_source: object,
        vad_engine: SileroVadEngine,
        asr_engine: SenseVoiceEngine,
        max_wait_seconds: float = 15.0,
    ) -> None:
        """
        Args:
            audio_source:
                实时麦克风输入对象。
            vad_engine:
                Silero VAD检测引擎。
            asr_engine:
                SenseVoice识别引擎。
            max_wait_seconds:
                每轮最长等待时间。超过该时间后自动结束。
        """
        self._audio_source = audio_source
        self._vad_engine = vad_engine
        self._asr_engine = asr_engine
        self._max_wait_seconds = float(max_wait_seconds)

        self._validate_configuration()

        # cancel_event用于从其他线程取消当前监听。
        self._cancel_event = Event()

        # state_lock只保护_running状态。
        self._state_lock = Lock()
        self._running = False

    @property
    def is_running(self) -> bool:
        """当前是否正在执行一轮监听。"""
        with self._state_lock:
            return self._running

    def run_once(
        self,
        *,
        max_wait_seconds: float | None = None,
    ) -> ListeningSessionResult:
        """
        开启一轮监听，只识别第一句完整语音。

        Returns:
            ListeningSessionResult:
                可能状态包括：
                - recognized：识别成功；
                - empty_result：检测到语音但ASR文本为空；
                - timeout：等待超时且未检测到有效语音；
                - cancelled：被外部取消。
        """
        wait_limit = (
            self._max_wait_seconds
            if max_wait_seconds is None
            else float(max_wait_seconds)
        )

        if wait_limit <= 0:
            raise ValueError(
                "max_wait_seconds必须大于0"
            )

        self._begin_session()
        session_start = monotonic()

        # 每轮开始前必须清除上一轮VAD状态。
        self._vad_engine.reset()

        try:
            self._audio_source.start()

            # 防止cancel()恰好发生在start()之前。
            if self._cancel_event.is_set():
                return self._cancelled_result(
                    session_start
                )

            while True:
                if self._cancel_event.is_set():
                    return self._cancelled_result(
                        session_start
                    )

                elapsed = monotonic() - session_start

                if elapsed >= wait_limit:
                    return self._finish_on_timeout(
                        session_start
                    )

                audio_chunk = self._audio_source.read()

                # stop()中断arecord后，read()会返回空数组。
                if audio_chunk.size == 0:
                    if self._cancel_event.is_set():
                        return self._cancelled_result(
                            session_start
                        )

                    continue

                segments = self._vad_engine.accept(
                    audio_chunk
                )

                if not segments:
                    continue

                # 一次True只识别一句话。
                # 即使本批音频中出现多个片段，也只取第一个。
                segment = segments[0]

                # 先关闭麦克风，再进行ASR。
                # 避免识别期间麦克风仍然持续采集。
                self._audio_source.stop()

                if self._cancel_event.is_set():
                    return self._cancelled_result(
                        session_start
                    )

                return self._recognize_segment(
                    segment=segment,
                    session_start=session_start,
                )

        finally:
            # 无论成功、超时、取消还是异常，都必须释放麦克风。
            self._audio_source.stop()

            # 清除未完成语音，防止污染下一轮。
            self._vad_engine.reset()

            self._end_session()

    def cancel(self) -> None:
        """
        立即取消当前监听。

        后续收到：
        /ai_agent/listen_control = False

        时，ROS2节点会调用此方法。
        """
        self._cancel_event.set()

        # stop()会终止arecord，使正在阻塞的read()尽快返回。
        self._audio_source.stop()

    def _recognize_segment(
        self,
        *,
        segment: VadSegment,
        session_start: float,
    ) -> ListeningSessionResult:
        """识别VAD已经检测完成的一句话。"""
        asr_result = self._asr_engine.recognize(
            samples=segment.samples,
            sample_rate=segment.sample_rate,
        )

        text = asr_result.text.strip()
        elapsed = monotonic() - session_start

        if not text:
            return ListeningSessionResult(
                status=ListeningStatus.EMPTY_RESULT,
                text="",
                elapsed_time=elapsed,
                vad_segment=segment,
                asr_result=asr_result,
            )

        return ListeningSessionResult(
            status=ListeningStatus.RECOGNIZED,
            text=text,
            elapsed_time=elapsed,
            vad_segment=segment,
            asr_result=asr_result,
        )

    def _finish_on_timeout(
        self,
        session_start: float,
    ) -> ListeningSessionResult:
        """
        达到等待上限时结束本轮。

        如果用户已经开始说话，但还没产生足够的结束静音，
        flush()会尝试输出当前有效语音。
        """
        self._audio_source.stop()

        if self._cancel_event.is_set():
            return self._cancelled_result(
                session_start
            )

        segments = self._vad_engine.flush()

        if segments:
            return self._recognize_segment(
                segment=segments[0],
                session_start=session_start,
            )

        return ListeningSessionResult(
            status=ListeningStatus.TIMEOUT,
            text="",
            elapsed_time=(
                monotonic() - session_start
            ),
        )

    def _cancelled_result(
        self,
        session_start: float,
    ) -> ListeningSessionResult:
        """构造取消结果。"""
        return ListeningSessionResult(
            status=ListeningStatus.CANCELLED,
            text="",
            elapsed_time=(
                monotonic() - session_start
            ),
        )

    def _begin_session(self) -> None:
        """原子地将会话设置为运行状态。"""
        with self._state_lock:
            if self._running:
                raise RuntimeError(
                    "当前已经有一轮监听正在运行"
                )

            self._running = True
            self._cancel_event.clear()

    def _end_session(self) -> None:
        """将会话恢复为非运行状态。"""
        with self._state_lock:
            self._running = False

    def _validate_configuration(self) -> None:
        """检查三个模块之间的关键参数是否一致。"""
        if self._max_wait_seconds <= 0:
            raise ValueError(
                "max_wait_seconds必须大于0"
            )

        sample_rates = {
            self._audio_source.sample_rate,
            self._vad_engine.sample_rate,
            self._asr_engine.sample_rate,
        }

        if len(sample_rates) != 1:
            raise ValueError(
                "音频源、VAD和ASR采样率必须一致，"
                f"当前分别为：{sorted(sample_rates)}"
            )
