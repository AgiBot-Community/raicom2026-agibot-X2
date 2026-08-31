#!/usr/bin/env python3

"""
Silero 语音活动检测引擎。

本模块只负责：
1. 加载 Silero VAD 模型；
2. 连续接收一维 float32 音频；
3. 自动按固定窗口送入 VAD；
4. 检测出完整语音片段；
5. 支持结束刷新和状态重置。

本模块不负责：
- ROS2 话题；
- 麦克风采集；
- ASR 文字识别；
- 意图识别；
- 状态机。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from threading import Lock

import numpy as np
import sherpa_onnx


@dataclass(frozen=True)
class VadSegment:
    """VAD检测出的一段完整语音。"""

    samples: np.ndarray
    start_sample: int
    sample_rate: int

    @property
    def end_sample(self) -> int:
        """语音段结束采样点。"""
        return self.start_sample + len(self.samples)

    @property
    def start_time(self) -> float:
        """语音段开始时间，单位为秒。"""
        return self.start_sample / self.sample_rate

    @property
    def end_time(self) -> float:
        """语音段结束时间，单位为秒。"""
        return self.end_sample / self.sample_rate

    @property
    def duration(self) -> float:
        """语音段持续时间，单位为秒。"""
        return len(self.samples) / self.sample_rate


class SileroVadEngine:
    """适用于连续音频流的 Silero VAD 引擎。"""

    def __init__(
        self,
        model_path: str | Path,
        *,
        sample_rate: int = 16000,
        threshold: float = 0.2,
        min_silence_duration: float = 0.5,
        min_speech_duration: float = 0.25,
        max_speech_duration: float = 10.0,
        buffer_size_in_seconds: int = 30,
    ) -> None:
        """
        创建 Silero VAD 引擎。

        Args:
            model_path:
                silero_vad.onnx 文件路径。
            sample_rate:
                音频采样率，当前固定使用 16000 Hz。
            threshold:
                语音概率阈值。
            min_silence_duration:
                连续静音达到该时长后，认为一句话结束。
            min_speech_duration:
                小于该时长的声音不作为有效语音。
            max_speech_duration:
                单段语音允许的最长时间。
            buffer_size_in_seconds:
                VAD内部最多缓存的音频时长。
        """
        self._model_path = Path(model_path).expanduser().resolve()
        self._sample_rate = int(sample_rate)
        self._threshold = float(threshold)
        self._min_silence_duration = float(
            min_silence_duration
        )
        self._min_speech_duration = float(
            min_speech_duration
        )
        self._max_speech_duration = float(
            max_speech_duration
        )
        self._buffer_size_in_seconds = int(
            buffer_size_in_seconds
        )

        self._validate_configuration()

        self._config = self._create_config()
        self._window_size = int(
            self._config.silero_vad.window_size
        )

        # 外部输入的音频块不一定正好是512个采样点。
        # 不足一个窗口的数据暂存在这里。
        self._pending_samples = np.empty(
            0,
            dtype=np.float32,
        )

        # 防止ROS2回调和音频线程同时操作同一个VAD。
        self._lock = Lock()

        self._detector = self._create_detector()

    @property
    def sample_rate(self) -> int:
        """返回VAD使用的采样率。"""
        return self._sample_rate

    @property
    def window_size(self) -> int:
        """返回Silero要求的固定窗口大小。"""
        return self._window_size

    @property
    def model_path(self) -> Path:
        """返回VAD模型文件路径。"""
        return self._model_path

    def accept(
        self,
        samples: np.ndarray,
    ) -> tuple[VadSegment, ...]:
        """
        接收一块连续音频，并返回已经完成的语音段。

        输入块可以是任意长度，不必正好为512个采样点。
        未满一个窗口的数据会保留到下一次调用。
        """
        audio = self._prepare_audio(samples)

        if audio.size == 0:
            return ()

        with self._lock:
            if self._pending_samples.size > 0:
                audio = np.concatenate(
                    (self._pending_samples, audio)
                )

            segments: list[VadSegment] = []
            offset = 0

            while (
                offset + self._window_size
                <= audio.size
            ):
                window = audio[
                    offset : offset + self._window_size
                ]

                self._detector.accept_waveform(window)
                offset += self._window_size

                segments.extend(
                    self._drain_completed_segments()
                )

            # 保留最后不足512点的部分。
            self._pending_samples = np.ascontiguousarray(
                audio[offset:],
                dtype=np.float32,
            )

            return tuple(segments)

    def flush(self) -> tuple[VadSegment, ...]:
        """
        通知VAD当前音频输入已经结束。

        如果仍有未满一个窗口的数据，会补零后送入VAD。
        如果用户正在说话但尚未形成结束静音，flush也会强制
        输出当前有效语音段。
        """
        with self._lock:
            if self._pending_samples.size > 0:
                padded = np.zeros(
                    self._window_size,
                    dtype=np.float32,
                )

                count = self._pending_samples.size
                padded[:count] = self._pending_samples

                self._detector.accept_waveform(padded)

                self._pending_samples = np.empty(
                    0,
                    dtype=np.float32,
                )

            self._detector.flush()

            return tuple(
                self._drain_completed_segments()
            )

    def reset(self) -> None:
        """
        清空当前VAD状态。

        当收到 listen_control=False、取消本轮监听或一句话
        已经处理完成后调用，防止上一轮残留音频进入下一轮。
        """
        with self._lock:
            self._detector = self._create_detector()

            self._pending_samples = np.empty(
                0,
                dtype=np.float32,
            )

    def _create_config(
        self,
    ) -> sherpa_onnx.VadModelConfig:
        """建立Silero VAD配置。"""
        config = sherpa_onnx.VadModelConfig()

        config.silero_vad.model = str(
            self._model_path
        )

        config.silero_vad.threshold = (
            self._threshold
        )

        config.silero_vad.min_silence_duration = (
            self._min_silence_duration
        )

        config.silero_vad.min_speech_duration = (
            self._min_speech_duration
        )

        config.silero_vad.max_speech_duration = (
            self._max_speech_duration
        )

        config.sample_rate = self._sample_rate

        return config

    def _create_detector(
        self,
    ) -> sherpa_onnx.VoiceActivityDetector:
        """根据当前配置创建新的VAD实例。"""
        return sherpa_onnx.VoiceActivityDetector(
            self._config,
            buffer_size_in_seconds=(
                self._buffer_size_in_seconds
            ),
        )

    def _drain_completed_segments(
        self,
    ) -> list[VadSegment]:
        """取出VAD队列中已经完成的全部语音段。"""
        segments: list[VadSegment] = []

        while not self._detector.empty():
            raw_segment = self._detector.front

            samples = np.asarray(
                raw_segment.samples,
                dtype=np.float32,
            ).copy()

            segment = VadSegment(
                samples=samples,
                start_sample=int(raw_segment.start),
                sample_rate=self._sample_rate,
            )

            segments.append(segment)
            self._detector.pop()

        return segments

    def _prepare_audio(
        self,
        samples: np.ndarray,
    ) -> np.ndarray:
        """检查并规范输入音频。"""
        audio = np.asarray(samples)

        if audio.ndim != 1:
            raise ValueError(
                "VAD输入必须是一维单声道数组，"
                f"当前数组形状为 {audio.shape}"
            )

        if audio.size == 0:
            return np.empty(0, dtype=np.float32)

        if not np.issubdtype(
            audio.dtype,
            np.floating,
        ):
            raise TypeError(
                "VAD输入必须是浮点音频数组，"
                f"当前数据类型为 {audio.dtype}"
            )

        audio = np.ascontiguousarray(
            audio,
            dtype=np.float32,
        )

        if not np.all(np.isfinite(audio)):
            raise ValueError(
                "VAD输入中含有NaN或无穷值"
            )

        peak = float(np.max(np.abs(audio)))

        if peak > 1.01:
            raise ValueError(
                "VAD输入音频幅度超出正常范围，"
                f"当前绝对峰值为 {peak:.4f}"
            )

        return audio

    def _validate_configuration(self) -> None:
        """检查模型文件和参数是否合法。"""
        if not self._model_path.is_file():
            raise FileNotFoundError(
                f"Silero VAD模型不存在："
                f"{self._model_path}"
            )

        if self._model_path.stat().st_size == 0:
            raise RuntimeError(
                f"Silero VAD模型是空文件："
                f"{self._model_path}"
            )

        if self._sample_rate != 16000:
            raise ValueError(
                "当前Silero VAD只使用16000 Hz，"
                f"实际配置为 {self._sample_rate} Hz"
            )

        if not 0.0 < self._threshold < 1.0:
            raise ValueError(
                "threshold必须处于0和1之间"
            )

        if self._min_silence_duration < 0:
            raise ValueError(
                "min_silence_duration不能小于0"
            )

        if self._min_speech_duration < 0:
            raise ValueError(
                "min_speech_duration不能小于0"
            )

        if self._max_speech_duration <= 0:
            raise ValueError(
                "max_speech_duration必须大于0"
            )

        if self._buffer_size_in_seconds <= 0:
            raise ValueError(
                "buffer_size_in_seconds必须大于0"
            )
