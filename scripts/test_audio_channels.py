#!/usr/bin/env python3

import math
import time
import wave
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)

from aimdk_msgs.msg import AudioCapture


CAPTURE_TOPIC = "/aima/hal/audio/capture"

QUIET_SECONDS = 4.0
SPEECH_SECONDS = 8.0

OUTPUT_DIR = (
    Path.home()
    / "star_agibot"
    / "audio_channel_test"
)


class ChannelStats:

    def __init__(self) -> None:
        self.sum_square = 0.0
        self.count = 0
        self.peak = 0.0

    def add(
        self,
        samples: np.ndarray,
    ) -> None:
        if samples.size == 0:
            return

        values = samples.astype(
            np.float64,
            copy=False,
        )

        self.sum_square += float(
            np.sum(values * values)
        )

        self.count += int(values.size)

        self.peak = max(
            self.peak,
            float(np.max(np.abs(values))),
        )

    @property
    def rms(self) -> float:
        if self.count == 0:
            return 0.0

        return math.sqrt(
            self.sum_square / self.count
        )

    @property
    def dbfs(self) -> float:
        if self.rms <= 0.0:
            return -120.0

        return 20.0 * math.log10(
            self.rms / 32768.0
        )


class AudioChannelTest(Node):

    def __init__(self) -> None:
        super().__init__("audio_channel_test")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            AudioCapture,
            CAPTURE_TOPIC,
            self.audio_callback,
            qos,
        )

        self.started_at = None
        self.last_phase = None
        self.done = False
        self.received_packets = 0

        self.channels = None
        self.mic_channels = None
        self.ref_channels = None
        self.sample_rate = None

        self.quiet_stats = []
        self.speech_stats = []
        self.speech_buffers = []

        print(
            f"等待音频话题：{CAPTURE_TOPIC}"
        )
        print()
        print(
            f"开始后前{QUIET_SECONDS:.0f}秒请保持安静。"
        )

    def initialize_format(
        self,
        msg: AudioCapture,
        channels: int,
    ) -> None:
        self.channels = channels
        self.mic_channels = int(
            msg.mic_channels
        )
        self.ref_channels = int(
            msg.ref_channels
        )
        self.sample_rate = int(
            msg.info.sample_rate
        )

        self.quiet_stats = [
            ChannelStats()
            for _ in range(channels)
        ]

        self.speech_stats = [
            ChannelStats()
            for _ in range(channels)
        ]

        self.speech_buffers = [
            []
            for _ in range(channels)
        ]

        print()
        print("========== 检测到音频格式 ==========")
        print("channels      =", channels)
        print("mic_channels  =", self.mic_channels)
        print("ref_channels  =", self.ref_channels)
        print("sample_rate   =", self.sample_rate)
        print(
            "sample_format =",
            msg.info.sample_format,
        )
        print(
            "coding_format =",
            msg.info.coding_format,
        )
        print()

    def audio_callback(
        self,
        msg: AudioCapture,
    ) -> None:
        raw = bytes(msg.data.data)

        if not raw:
            return

        channels = int(msg.info.channels)

        if channels <= 0:
            channels = (
                int(msg.mic_channels)
                + int(msg.ref_channels)
            )

        if channels <= 0:
            return

        samples = np.frombuffer(
            raw,
            dtype="<i2",
        )

        usable = (
            samples.size
            - samples.size % channels
        )

        if usable <= 0:
            return

        frames = samples[:usable].reshape(
            -1,
            channels,
        )

        if self.channels is None:
            self.initialize_format(
                msg,
                channels,
            )

        if channels != self.channels:
            print(
                "错误：音频声道数量在测试过程中发生变化"
            )
            self.done = True
            return

        now = time.monotonic()

        if self.started_at is None:
            self.started_at = now

        elapsed = now - self.started_at
        self.received_packets += 1

        if elapsed < QUIET_SECONDS:
            phase = "quiet"

            if self.last_phase != phase:
                print(
                    "【安静阶段】请不要说话……"
                )
                self.last_phase = phase

            for index in range(channels):
                self.quiet_stats[index].add(
                    frames[:, index]
                )

            return

        if elapsed < (
            QUIET_SECONDS
            + SPEECH_SECONDS
        ):
            phase = "speech"

            if self.last_phase != phase:
                print()
                print(
                    "【说话阶段】现在请靠近机器人，"
                    "以正常音量连续说话。"
                )
                print(
                    "建议重复："
                    "“机器人你好，请问现在是什么时间。”"
                )
                print()
                self.last_phase = phase

            for index in range(channels):
                channel_data = frames[
                    :,
                    index,
                ]

                self.speech_stats[index].add(
                    channel_data
                )

                self.speech_buffers[index].append(
                    channel_data.copy()
                )

            return

        self.done = True

    def save_wav_files(self) -> None:
        OUTPUT_DIR.mkdir(
            parents=True,
            exist_ok=True,
        )

        for index in range(
            int(self.mic_channels)
        ):
            if not self.speech_buffers[index]:
                continue

            data = np.concatenate(
                self.speech_buffers[index]
            ).astype(
                "<i2",
                copy=False,
            )

            output = (
                OUTPUT_DIR
                / f"mic_ch{index}_speech.wav"
            )

            with wave.open(
                str(output),
                "wb",
            ) as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(
                    int(self.sample_rate)
                )
                wav_file.writeframes(
                    data.tobytes()
                )

    def report(self) -> None:
        print()
        print("========== 声道测试结果 ==========")
        print(
            "数值中的ΔdB越大，"
            "说明说话相对环境噪声越明显。"
        )
        print()

        scores = []

        for index in range(
            int(self.channels)
        ):
            quiet = self.quiet_stats[index]
            speech = self.speech_stats[index]

            delta_db = (
                speech.dbfs
                - quiet.dbfs
            )

            channel_type = (
                "MIC"
                if index
                < int(self.mic_channels)
                else "REF"
            )

            print(
                f"ch{index} [{channel_type}] "
                f"安静={quiet.dbfs:7.2f} dBFS  "
                f"说话={speech.dbfs:7.2f} dBFS  "
                f"Δ={delta_db:6.2f} dB  "
                f"SpeechPeak={speech.peak:7.0f}"
            )

            if index < int(self.mic_channels):
                scores.append(
                    (
                        delta_db,
                        speech.dbfs,
                        speech.peak,
                        index,
                    )
                )

        if not scores:
            print("错误：没有可用的麦克风声道")
            return

        scores.sort(
            reverse=True
        )

        (
            best_delta,
            best_speech_db,
            best_peak,
            best_channel,
        ) = scores[0]

        print()
        print("========== 推荐结果 ==========")
        print(
            f"当前环境推荐声道：ch{best_channel}"
        )
        print(
            f"该声道语音增量：{best_delta:.2f} dB"
        )
        print(
            f"该声道原始语音峰值：{best_peak:.0f}"
        )

        if best_peak > 0:
            max_safe_gain = (
                28000.0 / best_peak
            )

            recommended_gain = min(
                16.0,
                max(
                    1.0,
                    max_safe_gain,
                ),
            )

            print(
                "按峰值估算的安全数字增益上限："
                f"{max_safe_gain:.2f}"
            )

            print(
                "建议初始数字增益："
                f"{recommended_gain:.1f}"
            )

            if best_peak * 16.0 >= 30000:
                print(
                    "警告：继续使用16倍增益可能削波，"
                    "应适当降低。"
                )
            else:
                print(
                    "使用16倍增益暂时不会因峰值"
                    "直接发生严重削波。"
                )

        print()

        if best_delta >= 6.0:
            print(
                "信噪表现：较好，适合直接用于VAD和ASR。"
            )
        elif best_delta >= 3.0:
            print(
                "信噪表现：基本可用，建议近距离说话。"
            )
        elif best_delta >= 1.0:
            print(
                "信噪表现：偏弱，VAD可能漏检。"
            )
        else:
            print(
                "信噪表现：很弱，需检查说话距离、"
                "机器人朝向或环境噪声。"
            )

        print(
            "录音已保存到：",
            OUTPUT_DIR,
        )


def main() -> None:
    rclpy.init()
    node = AudioChannelTest()

    wall_deadline = (
        time.monotonic()
        + QUIET_SECONDS
        + SPEECH_SECONDS
        + 8.0
    )

    try:
        while (
            not node.done
            and time.monotonic()
            < wall_deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        if node.started_at is None:
            print(
                "错误：没有收到任何音频消息。"
            )
            raise SystemExit(1)

        if not node.done:
            print(
                "错误：测试超时，音频数据可能中断。"
            )
            raise SystemExit(1)

        node.save_wav_files()
        node.report()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
