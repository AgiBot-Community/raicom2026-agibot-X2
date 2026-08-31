#!/usr/bin/env python3

"""使用 AimDK 上游 VAD 分段的单轮识别会话。"""

from __future__ import annotations

from threading import Event, Lock
from time import monotonic

import numpy as np

from .asr_engine import SenseVoiceEngine
from .listening_session import (
    ListeningSessionResult,
    ListeningStatus,
)


class ProcessedListeningSession:
    """
    一次 listen_control=True 只识别第一条完整上游 VAD 语音段。

    本类不创建、不调用 Silero VAD。
    """

    def __init__(
        self,
        *,
        audio_source,
        asr_engine: SenseVoiceEngine,
        max_wait_seconds: float = 20.0,
    ) -> None:
        self._audio_source = audio_source
        self._asr_engine = asr_engine
        self._max_wait_seconds = float(max_wait_seconds)

        if self._max_wait_seconds <= 0:
            raise ValueError("max_wait_seconds必须大于0")

        self._cancel_event = Event()
        self._state_lock = Lock()
        self._running = False

    @property
    def is_running(self) -> bool:
        with self._state_lock:
            return self._running

    def run_once(
        self,
        *,
        max_wait_seconds: float | None = None,
    ) -> ListeningSessionResult:
        wait_limit = (
            self._max_wait_seconds
            if max_wait_seconds is None
            else float(max_wait_seconds)
        )

        if wait_limit <= 0:
            raise ValueError("max_wait_seconds必须大于0")

        self._begin_session()
        session_start = monotonic()

        try:
            self._audio_source.start()

            while True:
                if self._cancel_event.is_set():
                    return self._cancelled_result(session_start)

                elapsed = monotonic() - session_start
                if elapsed >= wait_limit:
                    return self._finish_on_timeout(session_start)

                samples = self._audio_source.read_utterance(
                    timeout=min(0.2, wait_limit - elapsed)
                )

                if samples.size == 0:
                    continue

                self._audio_source.stop()

                if self._cancel_event.is_set():
                    return self._cancelled_result(session_start)

                return self._recognize(
                    samples=samples,
                    session_start=session_start,
                )

        finally:
            self._audio_source.stop()
            self._end_session()

    def cancel(self) -> None:
        self._cancel_event.set()
        self._audio_source.stop()

    def _recognize(
        self,
        *,
        samples: np.ndarray,
        session_start: float,
    ) -> ListeningSessionResult:
        asr_result = self._asr_engine.recognize(
            samples=samples,
            sample_rate=self._audio_source.sample_rate,
        )

        text = asr_result.text.strip()
        elapsed = monotonic() - session_start

        if not text:
            return ListeningSessionResult(
                status=ListeningStatus.EMPTY_RESULT,
                text="",
                elapsed_time=elapsed,
                vad_segment=None,
                asr_result=asr_result,
            )

        return ListeningSessionResult(
            status=ListeningStatus.RECOGNIZED,
            text=text,
            elapsed_time=elapsed,
            vad_segment=None,
            asr_result=asr_result,
        )

    def _finish_on_timeout(
        self,
        session_start: float,
    ) -> ListeningSessionResult:
        partial = self._audio_source.flush_partial()

        if partial.size > 0:
            return self._recognize(
                samples=partial,
                session_start=session_start,
            )

        return ListeningSessionResult(
            status=ListeningStatus.TIMEOUT,
            text="",
            elapsed_time=monotonic() - session_start,
        )

    def _cancelled_result(
        self,
        session_start: float,
    ) -> ListeningSessionResult:
        return ListeningSessionResult(
            status=ListeningStatus.CANCELLED,
            text="",
            elapsed_time=monotonic() - session_start,
        )

    def _begin_session(self) -> None:
        with self._state_lock:
            if self._running:
                raise RuntimeError("当前已有一轮监听正在运行")
            self._running = True
            self._cancel_event.clear()

    def _end_session(self) -> None:
        with self._state_lock:
            self._running = False
