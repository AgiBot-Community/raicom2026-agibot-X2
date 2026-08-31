#!/usr/bin/env python3

"""
AimDK 处理后音频输入模块。

实机测试确认：
- /agent/process_audio_output 不是连续静音流；
- audio_vad_state=1 BEGIN 时开始发布；
- audio_vad_state=2 PROCESSING 时继续发布；
- audio_vad_state=3 END 后停止；
- audio_data 包长不固定（实测 2560 ~ 19840 bytes）。

因此本模块直接使用上游 VAD 分段，不再运行本地 Silero VAD。
"""

from __future__ import annotations

from collections import deque
from threading import Condition
from time import monotonic
from typing import Any

import numpy as np


VAD_NONE = 0
VAD_BEGIN = 1
VAD_PROCESSING = 2
VAD_END = 3


class ProcessedAudioSourceError(RuntimeError):
    """处理后音频消息异常。"""


class ProcessedAudioSource:
    """将 ProcessedAudioOutput 聚合为完整单句 float32 PCM。"""

    def __init__(
        self,
        *,
        sample_rate: int = 16000,
        min_speech_duration: float = 0.20,
        max_speech_duration: float = 12.0,
        repeated_begin_gap_seconds: float = 0.35,
    ) -> None:
        self._sample_rate = int(sample_rate)
        self._min_speech_samples = int(
            self._sample_rate * float(min_speech_duration)
        )
        self._max_speech_samples = int(
            self._sample_rate * float(max_speech_duration)
        )
        self._repeated_begin_gap_seconds = float(
            repeated_begin_gap_seconds
        )

        self._condition = Condition()
        self._active = False
        self._collecting = False
        self._active_stream_id: int | None = None
        self._buffer = bytearray()
        self._completed: deque[np.ndarray] = deque()
        self._error_message: str | None = None
        self._last_message_time: float | None = None

        self._validate_configuration()

    @property
    def sample_rate(self) -> int:
        return self._sample_rate

    @property
    def is_running(self) -> bool:
        with self._condition:
            return self._active

    def start(self) -> None:
        """
        开启一轮监听。

        关键策略：启动后必须等到新的 BEGIN 才开始收集。
        如果此时上游还在发送上一句话的 PROCESSING/END，全部忽略，
        避免上一轮尾音污染下一轮。
        """
        with self._condition:
            if self._active:
                raise ProcessedAudioSourceError(
                    "处理后音频源已经处于运行状态"
                )

            self._active = True
            self._collecting = False
            self._active_stream_id = None
            self._buffer.clear()
            self._completed.clear()
            self._error_message = None
            self._last_message_time = None
            self._condition.notify_all()

    def stop(self) -> None:
        """停止接收并清空未消费数据。"""
        with self._condition:
            self._active = False
            self._collecting = False
            self._active_stream_id = None
            self._buffer.clear()
            self._completed.clear()
            self._last_message_time = None
            self._condition.notify_all()

    def accept_message(self, message: Any) -> None:
        """由 ROS2 回调送入 ProcessedAudioOutput。"""
        with self._condition:
            if not self._active:
                return

        try:
            state = int(message.audio_vad_state.value)
            stream_id = int(message.stream_id)
            raw = bytes(message.audio_data)
        except Exception as exc:
            with self._condition:
                self._error_message = (
                    "无法解析ProcessedAudioOutput："
                    f"{exc}"
                )
                self._condition.notify_all()
            return

        if len(raw) % 2 != 0:
            with self._condition:
                self._error_message = (
                    "处理后PCM字节数不是2的整数倍："
                    f"{len(raw)}"
                )
                self._condition.notify_all()
            return

        now = monotonic()

        with self._condition:
            if not self._active:
                return

            gap = None
            if self._last_message_time is not None:
                gap = now - self._last_message_time
            self._last_message_time = now

            if state == VAD_BEGIN:
                self._handle_begin(
                    stream_id=stream_id,
                    raw=raw,
                    gap_seconds=gap,
                )

            elif state == VAD_PROCESSING:
                if (
                    self._collecting
                    and stream_id == self._active_stream_id
                ):
                    self._append_raw(raw)

            elif state == VAD_END:
                if (
                    self._collecting
                    and stream_id == self._active_stream_id
                ):
                    self._append_raw(raw)
                    self._finalize_current()

            # NONE 或未知状态直接忽略。

            self._condition.notify_all()

    def read_utterance(
        self,
        *,
        timeout: float = 0.2,
    ) -> np.ndarray:
        """读取一条由上游 BEGIN/END 划分完成的语音。"""
        with self._condition:
            if self._error_message is not None:
                message = self._error_message
                self._error_message = None
                raise ProcessedAudioSourceError(message)

            if not self._active:
                return np.empty(0, dtype=np.float32)

            if not self._completed:
                self._condition.wait(timeout=float(timeout))

            if self._error_message is not None:
                message = self._error_message
                self._error_message = None
                raise ProcessedAudioSourceError(message)

            if not self._active:
                return np.empty(0, dtype=np.float32)

            if not self._completed:
                return np.empty(0, dtype=np.float32)

            return self._completed.popleft()

    def flush_partial(self) -> np.ndarray:
        """超时时尝试取出当前未收到 END 的有效语音。"""
        with self._condition:
            if not self._collecting or not self._buffer:
                return np.empty(0, dtype=np.float32)

            samples = self._decode_pcm(bytes(self._buffer))

            self._collecting = False
            self._active_stream_id = None
            self._buffer.clear()

            if samples.size < self._min_speech_samples:
                return np.empty(0, dtype=np.float32)

            return samples

    def _handle_begin(
        self,
        *,
        stream_id: int,
        raw: bytes,
        gap_seconds: float | None,
    ) -> None:
        """处理 BEGIN，包括实测中偶发的重复 BEGIN。"""
        if not self._collecting:
            self._collecting = True
            self._active_stream_id = stream_id
            self._buffer.clear()
            self._append_raw(raw)
            return

        if stream_id != self._active_stream_id:
            # 同时出现其它 stream 时优先完成当前一句，不混流。
            return

        if (
            gap_seconds is not None
            and gap_seconds > self._repeated_begin_gap_seconds
        ):
            # 如果距离上一包已经明显断开，则把前一段视为可能丢失 END。
            self._finalize_current()
            self._collecting = True
            self._active_stream_id = stream_id
            self._buffer.clear()
            self._append_raw(raw)
            return

        # 短间隔重复 BEGIN 视为同一句的重复状态，不清空前文。
        self._append_raw(raw)

    def _append_raw(self, raw: bytes) -> None:
        if raw:
            self._buffer.extend(raw)

        current_samples = len(self._buffer) // 2
        if current_samples >= self._max_speech_samples:
            self._finalize_current()

    def _finalize_current(self) -> None:
        if not self._buffer:
            self._collecting = False
            self._active_stream_id = None
            return

        samples = self._decode_pcm(bytes(self._buffer))

        self._buffer.clear()
        self._collecting = False
        self._active_stream_id = None

        if samples.size < self._min_speech_samples:
            return

        self._completed.append(samples)

    @staticmethod
    def _decode_pcm(raw: bytes) -> np.ndarray:
        if not raw:
            return np.empty(0, dtype=np.float32)

        pcm = np.frombuffer(raw, dtype="<i2").astype(np.float32)
        samples = pcm / 32768.0
        return np.ascontiguousarray(samples, dtype=np.float32)

    def _validate_configuration(self) -> None:
        if self._sample_rate != 16000:
            raise ValueError("处理后音频当前固定使用16000 Hz")

        if self._min_speech_samples <= 0:
            raise ValueError("min_speech_duration必须大于0")

        if self._max_speech_samples <= self._min_speech_samples:
            raise ValueError("max_speech_duration必须大于最短语音时长")

        if self._repeated_begin_gap_seconds <= 0:
            raise ValueError("repeated_begin_gap_seconds必须大于0")
