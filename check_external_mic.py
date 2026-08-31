#!/usr/bin/env python3

import math
import time
from collections import Counter

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from aimdk_msgs.msg import (
    AudioCapture,
    ProcessedAudioOutput,
)


RAW_TOPIC = "/aima/hal/audio/capture"
PROCESSED_TOPIC = "/agent/process_audio_output"


class Stats:

    def __init__(self):
        self.reset()

    def reset(self):
        self.raw_count = 0

        self.channels = None
        self.mic_channels = None
        self.ref_channels = None
        self.sample_rate = None
        self.sample_format = None

        self.sum_sq = []
        self.sample_count = []
        self.peak = []

        self.processed_count = 0
        self.processed_bytes = []
        self.vad = Counter()

    def add_raw(self, msg):
        try:
            raw = bytes(msg.data.data)

            channels = int(msg.info.channels)
            mic_channels = int(msg.mic_channels)
            ref_channels = int(msg.ref_channels)

            pcm = np.frombuffer(
                raw,
                dtype="<i2",
            )

            if channels <= 0:
                return

            if pcm.size == 0:
                return

            if pcm.size % channels != 0:
                return

            pcm = pcm.reshape(
                -1,
                channels,
            )

        except Exception as exc:
            print(
                "RAW解析异常:",
                exc,
            )
            return

        if self.channels is None:
            self.channels = channels
            self.mic_channels = mic_channels
            self.ref_channels = ref_channels
            self.sample_rate = int(
                msg.info.sample_rate
            )
            self.sample_format = str(
                msg.info.sample_format
            )

            self.sum_sq = [
                0.0
                for _ in range(mic_channels)
            ]

            self.sample_count = [
                0
                for _ in range(mic_channels)
            ]

            self.peak = [
                0
                for _ in range(mic_channels)
            ]

        self.raw_count += 1

        for ch in range(mic_channels):

            x = pcm[:, ch].astype(
                np.float64
            )

            self.sum_sq[ch] += float(
                np.sum(x * x)
            )

            self.sample_count[ch] += int(
                x.size
            )

            if x.size:
                self.peak[ch] = max(
                    self.peak[ch],
                    int(
                        np.max(
                            np.abs(x)
                        )
                    ),
                )

    def add_processed(self, msg):
        self.processed_count += 1

        self.processed_bytes.append(
            len(msg.audio_data)
        )

        try:
            state = int(
                msg.audio_vad_state.value
            )
        except Exception:
            state = -1

        self.vad[state] += 1

    def report(self, title):
        print()
        print("=" * 60)
        print(title)
        print("=" * 60)

        print(
            "RAW消息数 =",
            self.raw_count,
        )

        if self.raw_count:

            print(
                "sample_rate =",
                self.sample_rate,
            )

            print(
                "sample_format =",
                self.sample_format,
            )

            print(
                "channels =",
                self.channels,
            )

            print(
                "mic_channels =",
                self.mic_channels,
            )

            print(
                "ref_channels =",
                self.ref_channels,
            )

            for ch in range(
                self.mic_channels or 0
            ):

                n = self.sample_count[ch]

                if n <= 0:
                    continue

                rms = math.sqrt(
                    self.sum_sq[ch]
                    / n
                )

                if rms > 0:
                    dbfs = (
                        20.0
                        * math.log10(
                            rms / 32768.0
                        )
                    )
                else:
                    dbfs = -120.0

                print(
                    f"mic ch{ch}: "
                    f"RMS={rms:.2f}, "
                    f"dBFS={dbfs:.2f}, "
                    f"peak={self.peak[ch]}"
                )

        print()
        print(
            "PROCESSED消息数 =",
            self.processed_count,
        )

        if self.processed_count:

            print(
                "PROCESSED包大小范围 =",
                min(
                    self.processed_bytes
                ),
                "~",
                max(
                    self.processed_bytes
                ),
                "bytes",
            )

            print(
                "PROCESSED VAD =",
                dict(self.vad),
            )


class CheckNode(Node):

    def __init__(self):
        super().__init__(
            "external_mic_check"
        )

        qos = QoSProfile(depth=100)

        qos.reliability = (
            ReliabilityPolicy.BEST_EFFORT
        )

        qos.durability = (
            DurabilityPolicy.VOLATILE
        )

        self.stats = Stats()

        self.create_subscription(
            AudioCapture,
            RAW_TOPIC,
            self.raw_cb,
            qos,
        )

        self.create_subscription(
            ProcessedAudioOutput,
            PROCESSED_TOPIC,
            self.processed_cb,
            qos,
        )

    def raw_cb(self, msg):
        self.stats.add_raw(msg)

    def processed_cb(self, msg):
        self.stats.add_processed(msg)


def collect(node, title, seconds):

    node.stats.reset()

    print()
    print(
        f">>> {title}（{seconds}秒）"
    )

    start = time.monotonic()

    while (
        time.monotonic()
        - start
        < seconds
    ):

        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

    node.stats.report(title)


def main():

    rclpy.init()

    node = CheckNode()

    try:

        collect(
            node,
            "阶段1：请完全安静",
            5,
        )

        collect(
            node,
            "阶段2：请对着外置麦正常连续说话",
            6,
        )

        collect(
            node,
            "阶段3：再次完全安静",
            5,
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
