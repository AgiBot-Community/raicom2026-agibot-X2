#!/usr/bin/env python3

"""
基于 ALSA arecord 的实时音频输入模块。

本模块只负责：
1. 启动 arecord；
2. 从标准输出读取原始 PCM 音频；
3. 将 16-bit PCM 转换为 float32；
4. 支持外部立即停止录音；
5. 检测 arecord 异常退出。

本模块不负责：
- VAD；
- ASR；
- ROS2 话题；
- 意图识别；
- 状态机。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path
from threading import Lock
from typing import BinaryIO

import numpy as np


class AudioSourceError(RuntimeError):
    """音频输入设备或 arecord 运行异常。"""


class ArecordAudioSource:
    """通过 arecord 获取实时单声道 PCM 音频。"""

    def __init__(
        self,
        *,
        device: str = "default",
        sample_rate: int = 16000,
        channels: int = 1,
        sample_width: int = 2,
        chunk_samples: int = 1600,
    ) -> None:
        """
        初始化 arecord 音频源。

        Args:
            device:
                ALSA录音设备名称，默认使用 default。
            sample_rate:
                采样率，当前使用16000 Hz。
            channels:
                声道数，当前只支持单声道。
            sample_width:
                每个采样点的字节数。
                16-bit PCM对应2字节。
            chunk_samples:
                每次向上层返回多少个采样点。
                默认1600点，即100毫秒音频。
        """
        self._device = str(device)
        self._sample_rate = int(sample_rate)
        self._channels = int(channels)
        self._sample_width = int(sample_width)
        self._chunk_samples = int(chunk_samples)

        self._validate_configuration()

        self._process: subprocess.Popen | None = None
        self._state_lock = Lock()
        self._stop_requested = False

    @property
    def device(self) -> str:
        """返回当前录音设备名称。"""
        return self._device

    @property
    def sample_rate(self) -> int:
        """返回采样率。"""
        return self._sample_rate

    @property
    def chunk_samples(self) -> int:
        """返回每次读取的采样点数。"""
        return self._chunk_samples

    @property
    def chunk_duration(self) -> float:
        """返回每块音频对应的时长，单位为秒。"""
        return self._chunk_samples / self._sample_rate

    @property
    def is_running(self) -> bool:
        """判断 arecord 是否仍在运行。"""
        with self._state_lock:
            process = self._process

        return (
            process is not None
            and process.poll() is None
        )

    def start(self) -> None:
        """
        启动 arecord。

        同一个音频源不能重复启动。
        """
        with self._state_lock:
            if (
                self._process is not None
                and self._process.poll() is None
            ):
                raise AudioSourceError(
                    "arecord 已经处于运行状态"
                )

            arecord_path = shutil.which("arecord")

            if arecord_path is None:
                raise AudioSourceError(
                    "系统中没有找到 arecord 命令"
                )

            command = [
                arecord_path,
                "-q",
                "-D",
                self._device,
                "-t",
                "raw",
                "-f",
                "S16_LE",
                "-r",
                str(self._sample_rate),
                "-c",
                str(self._channels),
            ]

            self._stop_requested = False

            self._process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )

            if self._process.stdout is None:
                process = self._process
                self._process = None
                process.terminate()

                raise AudioSourceError(
                    "无法读取 arecord 的标准输出"
                )

    def read(self) -> np.ndarray:
        """
        读取一块实时音频。

        Returns:
            一维 float32 数组，范围约为 [-1.0, 1.0]。

            如果外部主动调用 stop()，当前阻塞读取被终止，
            本方法返回空数组。
        """
        with self._state_lock:
            process = self._process
            stop_requested = self._stop_requested

        if stop_requested:
            return np.empty(0, dtype=np.float32)

        if process is None:
            raise AudioSourceError(
                "音频源尚未启动，请先调用 start()"
            )

        if process.stdout is None:
            raise AudioSourceError(
                "arecord 标准输出不可用"
            )

        bytes_to_read = (
            self._chunk_samples
            * self._channels
            * self._sample_width
        )

        raw_audio = self._read_exactly(
            process.stdout,
            bytes_to_read,
        )

        if len(raw_audio) != bytes_to_read:
            with self._state_lock:
                stop_requested = self._stop_requested

            if stop_requested:
                return np.empty(0, dtype=np.float32)

            error_message = self._read_process_error(process)

            raise AudioSourceError(
                "未能读取完整音频块。"
                f"期望 {bytes_to_read} 字节，"
                f"实际获得 {len(raw_audio)} 字节。"
                f"{error_message}"
            )

        pcm16 = np.frombuffer(
            raw_audio,
            dtype="<i2",
        )

        if self._channels > 1:
            pcm16 = pcm16.reshape(
                -1,
                self._channels,
            )

            # 当前预留多声道处理逻辑。
            # 正式配置仍然固定使用单声道。
            pcm16 = pcm16.mean(axis=1)

        samples = (
            pcm16.astype(np.float32)
            / 32768.0
        )

        return np.ascontiguousarray(
            samples,
            dtype=np.float32,
        )

    def stop(self) -> None:
        """
        停止 arecord。

        该方法允许被重复调用。
        当 read() 正在阻塞等待音频时，终止 arecord 会关闭管道，
        从而使 read() 尽快返回。
        """
        with self._state_lock:
            self._stop_requested = True
            process = self._process
            self._process = None

        if process is None:
            return

        if process.poll() is None:
            process.terminate()

            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)

        if process.stdout is not None:
            process.stdout.close()

        if process.stderr is not None:
            process.stderr.close()

    def __enter__(self) -> "ArecordAudioSource":
        """支持 with 语句自动启动。"""
        self.start()
        return self

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback,
    ) -> None:
        """离开 with 语句时自动关闭。"""
        self.stop()

    @staticmethod
    def _read_exactly(
        stream: BinaryIO,
        size: int,
    ) -> bytes:
        """
        从管道读取指定数量的字节。

        pipe.read() 不保证一次返回完整数据，因此需要循环读取。
        """
        data = bytearray()

        while len(data) < size:
            chunk = stream.read(size - len(data))

            if not chunk:
                break

            data.extend(chunk)

        return bytes(data)

    @staticmethod
    def _read_process_error(
        process: subprocess.Popen,
    ) -> str:
        """读取 arecord 异常退出时的错误信息。"""
        return_code = process.poll()

        if return_code is None:
            return ""

        stderr_text = ""

        if process.stderr is not None:
            try:
                stderr_data = process.stderr.read()
                stderr_text = stderr_data.decode(
                    "utf-8",
                    errors="replace",
                ).strip()
            except Exception:
                stderr_text = ""

        message = f"arecord 返回码：{return_code}。"

        if stderr_text:
            message += f"错误信息：{stderr_text}"

        return message

    def _validate_configuration(self) -> None:
        """检查录音参数是否合法。"""
        if not self._device:
            raise ValueError(
                "录音设备名称不能为空"
            )

        if self._sample_rate != 16000:
            raise ValueError(
                "当前语音链路固定使用16000 Hz，"
                f"实际配置为 {self._sample_rate} Hz"
            )

        if self._channels != 1:
            raise ValueError(
                "当前语音链路只支持单声道，"
                f"实际配置为 {self._channels} 声道"
            )

        if self._sample_width != 2:
            raise ValueError(
                "当前语音链路只支持16-bit PCM，"
                f"实际每个采样点为 {self._sample_width} 字节"
            )

        if self._chunk_samples <= 0:
            raise ValueError(
                "chunk_samples必须大于0"
            )
