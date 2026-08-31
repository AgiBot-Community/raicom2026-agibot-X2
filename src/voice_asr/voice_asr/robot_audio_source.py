#!/usr/bin/env python3

"""
机器人原始 ROS2 音频输入模块。

本模块同时支持两种已经实机验证的 AudioCapture 格式：

1. 内置麦：
   16000 Hz / S16LE / 6总声道 / 4麦克风 + 2参考；
   正式参数使用 mic channel 2，数字增益 14.0。

2. 外置麦：
   16000 Hz / S16LE / 2总声道 / 1麦克风 + 1参考；
   正式参数使用 mic channel 0，数字增益 1.0。

具体声道数、选择声道和增益均由 YAML 参数传入。
"""

from __future__ import annotations

from threading import Condition
from typing import Any

import numpy as np


class RobotAudioSourceError(RuntimeError):
    """机器人音频格式或数据异常。"""


class RobotAudioSource:
    """将 AudioCapture 原始交错 PCM 转换为单声道 float32。"""

    def __init__(
        self,
        *,
        sample_rate: int = 16000,
        chunk_samples: int = 1600,
        selected_channel: int = 0,
        digital_gain: float = 1.0,
        expected_total_channels: int = 2,
        expected_mic_channels: int = 1,
        expected_ref_channels: int = 1,
        max_buffer_seconds: float = 3.0,
    ) -> None:
        self._sample_rate = int(sample_rate)
        self._chunk_samples = int(chunk_samples)
        self._selected_channel = int(selected_channel)
        self._digital_gain = float(digital_gain)
        self._expected_total_channels = int(
            expected_total_channels
        )
        self._expected_mic_channels = int(
            expected_mic_channels
        )
        self._expected_ref_channels = int(
            expected_ref_channels
        )

        self._max_buffer_samples = int(
            self._sample_rate * float(max_buffer_seconds)
        )

        self._condition = Condition()
        self._active = False
        self._pending = np.empty(0, dtype=np.float32)
        self._error_message: str | None = None

        self._validate_configuration()

    @property
    def sample_rate(self) -> int:
        return self._sample_rate

    @property
    def chunk_samples(self) -> int:
        return self._chunk_samples

    @property
    def selected_channel(self) -> int:
        return self._selected_channel

    @property
    def digital_gain(self) -> float:
        return self._digital_gain

    @property
    def is_running(self) -> bool:
        with self._condition:
            return self._active

    def start(self) -> None:
        """开启一轮音频接收，并清空上一轮缓存。"""
        with self._condition:
            if self._active:
                raise RobotAudioSourceError(
                    "机器人原始音频源已经处于运行状态"
                )

            self._pending = np.empty(0, dtype=np.float32)
            self._error_message = None
            self._active = True
            self._condition.notify_all()

    def stop(self) -> None:
        """停止本轮接收并清空缓存。"""
        with self._condition:
            self._active = False
            self._pending = np.empty(0, dtype=np.float32)
            self._condition.notify_all()

    def accept_message(self, message: Any) -> None:
        """由 ROS2 订阅回调送入一条 AudioCapture。"""
        with self._condition:
            if not self._active:
                return

        try:
            samples = self._decode_message(message)
        except Exception as exc:
            with self._condition:
                self._error_message = str(exc)
                self._condition.notify_all()
            return

        if samples.size == 0:
            return

        with self._condition:
            if not self._active:
                return

            if self._pending.size == 0:
                self._pending = samples
            else:
                self._pending = np.concatenate(
                    (self._pending, samples)
                )

            if self._pending.size > self._max_buffer_samples:
                self._pending = self._pending[
                    -self._max_buffer_samples :
                ]

            self._condition.notify_all()

    def read(self) -> np.ndarray:
        """阻塞式读取一块单声道 float32 音频。"""
        with self._condition:
            if self._error_message is not None:
                message = self._error_message
                self._error_message = None
                raise RobotAudioSourceError(message)

            if not self._active:
                return np.empty(0, dtype=np.float32)

            if self._pending.size < self._chunk_samples:
                self._condition.wait(timeout=0.2)

            if self._error_message is not None:
                message = self._error_message
                self._error_message = None
                raise RobotAudioSourceError(message)

            if not self._active:
                return np.empty(0, dtype=np.float32)

            if self._pending.size < self._chunk_samples:
                return np.empty(0, dtype=np.float32)

            output = np.ascontiguousarray(
                self._pending[: self._chunk_samples],
                dtype=np.float32,
            )
            self._pending = self._pending[self._chunk_samples :]
            return output

    def _decode_message(self, message: Any) -> np.ndarray:
        """解析交错 S16LE PCM 并提取指定麦克风声道。"""
        info = message.info

        sample_rate = int(info.sample_rate)
        total_channels = int(info.channels)
        mic_channels = int(message.mic_channels)
        ref_channels = int(message.ref_channels)

        sample_format = self._normalize_text(info.sample_format)
        coding_format = self._normalize_text(info.coding_format)

        if sample_rate != self._sample_rate:
            raise RobotAudioSourceError(
                "机器人原始音频采样率不一致："
                f"期望{self._sample_rate} Hz，"
                f"实际{sample_rate} Hz"
            )

        if sample_format not in ("", "S16LE", "S16"):
            raise RobotAudioSourceError(
                "暂不支持音频格式："
                f"{info.sample_format!r}"
            )

        if coding_format not in ("", "PCM", "RAW"):
            raise RobotAudioSourceError(
                "暂不支持音频编码："
                f"{info.coding_format!r}"
            )

        if (
            total_channels != self._expected_total_channels
            or mic_channels != self._expected_mic_channels
            or ref_channels != self._expected_ref_channels
        ):
            raise RobotAudioSourceError(
                "当前 AudioCapture 与所选模式的已验证格式不一致："
                f"channels={total_channels}, "
                f"mic={mic_channels}, ref={ref_channels}；"
                "期望 channels="
                f"{self._expected_total_channels}, mic="
                f"{self._expected_mic_channels}, ref="
                f"{self._expected_ref_channels}"
            )

        if not (0 <= self._selected_channel < mic_channels):
            raise RobotAudioSourceError(
                "麦克风声道越界："
                f"selected_channel={self._selected_channel}, "
                f"mic_channels={mic_channels}"
            )

        raw = bytes(message.data.data)
        if not raw:
            return np.empty(0, dtype=np.float32)

        bytes_per_frame = total_channels * 2
        if len(raw) % bytes_per_frame != 0:
            raise RobotAudioSourceError(
                "原始音频长度不能被每帧字节数整除："
                f"len={len(raw)}, bytes_per_frame={bytes_per_frame}"
            )

        pcm = np.frombuffer(raw, dtype="<i2").reshape(
            -1,
            total_channels,
        )

        selected = pcm[:, self._selected_channel].astype(
            np.float32
        )

        samples = selected / 32768.0

        if self._digital_gain != 1.0:
            samples = samples * self._digital_gain
            samples = np.clip(samples, -1.0, 1.0)

        return np.ascontiguousarray(samples, dtype=np.float32)

    def _validate_configuration(self) -> None:
        if self._sample_rate != 16000:
            raise ValueError("当前机器人原始音频链路固定使用16000 Hz")

        if self._chunk_samples <= 0:
            raise ValueError("chunk_samples必须大于0")

        if self._selected_channel < 0:
            raise ValueError("selected_channel不能小于0")

        if self._digital_gain <= 0:
            raise ValueError("digital_gain必须大于0")

        if self._max_buffer_samples <= self._chunk_samples:
            raise ValueError("原始音频缓存过小")

    @staticmethod
    def _normalize_text(value: Any) -> str:
        return str(value).strip().upper()
