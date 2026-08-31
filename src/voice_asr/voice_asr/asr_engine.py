#!/usr/bin/env python3

"""
SenseVoice 离线语音识别引擎。

本模块只负责：
1. 检查模型和词表文件；
2. 启动时加载一次 SenseVoice；
3. 接收一维 float32 音频数组；
4. 输出最终中文文本和性能信息。

本模块不负责：
- ROS2 话题；
- 麦克风采集；
- VAD；
- 状态机；
- 意图识别。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from time import perf_counter
from typing import Any

import numpy as np
import sherpa_onnx


@dataclass(frozen=True)
class AsrResult:
    """一次语音识别的结构化结果。"""

    text: str
    decode_time: float
    audio_duration: float
    rtf: float
    raw_result: str


class SenseVoiceEngine:
    """SenseVoice INT8 离线识别引擎。"""

    def __init__(
        self,
        model_path: str | Path,
        tokens_path: str | Path,
        *,
        sample_rate: int = 16000,
        num_threads: int = 2,
        language: str = "zh",
        use_itn: bool = False,
        provider: str = "cpu",
        debug: bool = False,
    ) -> None:
        """
        创建并加载 SenseVoice 识别器。

        Args:
            model_path:
                model.int8.onnx 文件路径。
            tokens_path:
                tokens.txt 文件路径。
            sample_rate:
                模型输入采样率，当前固定使用 16000 Hz。
            num_threads:
                CPU 推理线程数。
            language:
                识别语言，当前使用 zh。
            use_itn:
                是否启用逆文本规范化。
            provider:
                ONNX Runtime 执行后端，当前使用 cpu。
            debug:
                是否输出 sherpa-onnx 底层调试日志。
        """
        self._model_path = Path(model_path).expanduser().resolve()
        self._tokens_path = Path(tokens_path).expanduser().resolve()

        self._sample_rate = int(sample_rate)
        self._num_threads = int(num_threads)
        self._language = str(language)
        self._use_itn = bool(use_itn)
        self._provider = str(provider)
        self._debug = bool(debug)

        self._validate_configuration()

        # sherpa-onnx 的识别器和 stream 调用放在锁内，
        # 避免未来多个线程同时调用同一个引擎。
        self._decode_lock = Lock()

        load_start = perf_counter()

        self._recognizer = (
            sherpa_onnx.OfflineRecognizer.from_sense_voice(
                model=str(self._model_path),
                tokens=str(self._tokens_path),
                sample_rate=self._sample_rate,
                num_threads=self._num_threads,
                language=self._language,
                use_itn=self._use_itn,
                provider=self._provider,
                debug=self._debug,
            )
        )

        self._load_time = perf_counter() - load_start

    @property
    def sample_rate(self) -> int:
        """返回模型要求的采样率。"""
        return self._sample_rate

    @property
    def load_time(self) -> float:
        """返回模型初始化耗时，单位为秒。"""
        return self._load_time

    @property
    def model_path(self) -> Path:
        """返回模型文件路径。"""
        return self._model_path

    @property
    def tokens_path(self) -> Path:
        """返回词表文件路径。"""
        return self._tokens_path

    def recognize(
        self,
        samples: np.ndarray,
        sample_rate: int,
    ) -> AsrResult:
        """
        识别一段完整语音。

        Args:
            samples:
                一维 float32 音频数组，数值范围约为 [-1.0, 1.0]。
            sample_rate:
                当前音频采样率，必须与模型采样率一致。

        Returns:
            AsrResult:
                包含最终文本、识别耗时、音频时长和 RTF。
        """
        audio = self._prepare_audio(samples, sample_rate)

        audio_duration = len(audio) / self._sample_rate

        with self._decode_lock:
            stream = self._recognizer.create_stream()
            stream.accept_waveform(self._sample_rate, audio)

            decode_start = perf_counter()
            self._recognizer.decode_stream(stream)
            decode_time = perf_counter() - decode_start

            result: Any = stream.result

        text = str(getattr(result, "text", "")).strip()

        rtf = (
            decode_time / audio_duration
            if audio_duration > 0
            else 0.0
        )

        return AsrResult(
            text=text,
            decode_time=decode_time,
            audio_duration=audio_duration,
            rtf=rtf,
            raw_result=str(result),
        )

    def _validate_configuration(self) -> None:
        """检查模型配置和文件路径。"""
        self._require_file(self._model_path, "SenseVoice 模型")
        self._require_file(self._tokens_path, "SenseVoice 词表")

        if self._sample_rate != 16000:
            raise ValueError(
                "当前 SenseVoice 引擎只接受 16000 Hz，"
                f"实际配置为 {self._sample_rate} Hz"
            )

        if self._num_threads < 1:
            raise ValueError("num_threads 必须大于或等于 1")

        if not self._language:
            raise ValueError("language 不能为空")

        if not self._provider:
            raise ValueError("provider 不能为空")

    def _prepare_audio(
        self,
        samples: np.ndarray,
        sample_rate: int,
    ) -> np.ndarray:
        """检查并规范识别输入音频。"""
        if int(sample_rate) != self._sample_rate:
            raise ValueError(
                f"音频采样率必须是 {self._sample_rate} Hz，"
                f"实际为 {sample_rate} Hz"
            )

        audio = np.asarray(samples)

        if audio.ndim != 1:
            raise ValueError(
                "识别输入必须是一维单声道数组，"
                f"当前数组形状为 {audio.shape}"
            )

        if audio.size == 0:
            raise ValueError("识别输入不能为空")

        if not np.issubdtype(audio.dtype, np.floating):
            raise TypeError(
                "识别输入必须是浮点音频数组，"
                f"当前数据类型为 {audio.dtype}"
            )

        # 统一为 sherpa-onnx 使用的 float32。
        audio = np.ascontiguousarray(audio, dtype=np.float32)

        if not np.all(np.isfinite(audio)):
            raise ValueError("音频中含有 NaN 或无穷值")

        peak = float(np.max(np.abs(audio)))

        # 正常归一化音频应处于 [-1, 1]。
        # 保留少量数值误差余量。
        if peak > 1.01:
            raise ValueError(
                "音频幅度超出正常归一化范围，"
                f"当前绝对峰值为 {peak:.4f}"
            )

        return audio

    @staticmethod
    def _require_file(path: Path, description: str) -> None:
        """检查文件存在且非空。"""
        if not path.is_file():
            raise FileNotFoundError(
                f"{description}不存在：{path}"
            )

        if path.stat().st_size == 0:
            raise RuntimeError(
                f"{description}是空文件：{path}"
            )
