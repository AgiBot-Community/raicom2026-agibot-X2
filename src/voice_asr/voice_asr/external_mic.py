#!/usr/bin/env python3

"""AimDK 麦克风源切换与确认工具。"""

from __future__ import annotations

import time
from typing import Any

import rclpy


INTERNAL_MIC_SOURCE_ID = 0
EXTERNAL_MIC_SOURCE_ID = 1


class MicSourceError(RuntimeError):
    """无法切换或确认麦克风源。"""


# 兼容上一版代码中的异常名称。
ExternalMicError = MicSourceError


def _status_value(response: Any) -> int | None:
    """读取 AimDK CommonResponse.status.value。"""
    try:
        return int(response.header.status.value)
    except Exception:
        return None


def _wait_future(node, future, timeout_sec: float) -> Any:
    """在节点进入主 spin 前等待一次服务结果。"""
    deadline = time.monotonic() + float(timeout_sec)

    while rclpy.ok() and not future.done():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break

        rclpy.spin_once(
            node,
            timeout_sec=min(0.1, remaining),
        )

    if not future.done():
        raise MicSourceError("等待麦克风服务响应超时")

    exception = future.exception()
    if exception is not None:
        raise MicSourceError(
            f"麦克风服务调用异常：{exception}"
        )

    response = future.result()
    if response is None:
        raise MicSourceError("麦克风服务没有返回响应")

    return response


def ensure_mic_source(
    node,
    *,
    target_source_id: int,
    set_service_name: str,
    get_service_name: str,
    timeout_sec: float = 5.0,
) -> int:
    """
    将 AimDK 麦克风源切换到指定来源并再次读取确认。

    mic_source：
        0 = 内置麦
        1 = 外置麦
    """
    try:
        from aimdk_msgs.srv import (
            GetMicSourceRequest,
            SetMicSourceRequest,
        )
    except ImportError as exc:
        raise MicSourceError(
            "无法导入 AimDK 麦克风源服务接口"
        ) from exc

    target_source_id = int(target_source_id)
    if target_source_id not in (
        INTERNAL_MIC_SOURCE_ID,
        EXTERNAL_MIC_SOURCE_ID,
    ):
        raise MicSourceError(
            f"不支持的 mic_source={target_source_id}"
        )

    set_client = node.create_client(
        SetMicSourceRequest,
        set_service_name,
    )

    if not set_client.wait_for_service(
        timeout_sec=float(timeout_sec)
    ):
        raise MicSourceError(
            "麦克风切换服务不可用："
            f"{set_service_name}"
        )

    set_request = SetMicSourceRequest.Request()
    set_request.mic_source = target_source_id

    set_response = _wait_future(
        node,
        set_client.call_async(set_request),
        timeout_sec,
    )

    # AimDK CommonState: SUCCESS = 1。
    set_status = _status_value(set_response)
    if set_status not in (None, 1):
        message = str(
            getattr(set_response.header, "message", "")
        )
        raise MicSourceError(
            "切换麦克风源失败："
            f"status={set_status}, message={message!r}"
        )

    get_client = node.create_client(
        GetMicSourceRequest,
        get_service_name,
    )

    if not get_client.wait_for_service(
        timeout_sec=float(timeout_sec)
    ):
        raise MicSourceError(
            "麦克风源查询服务不可用："
            f"{get_service_name}"
        )

    get_request = GetMicSourceRequest.Request()
    get_response = _wait_future(
        node,
        get_client.call_async(get_request),
        timeout_sec,
    )

    get_status = _status_value(get_response)
    if get_status not in (None, 1):
        message = str(
            getattr(get_response.header, "message", "")
        )
        raise MicSourceError(
            "查询麦克风源失败："
            f"status={get_status}, message={message!r}"
        )

    actual_source = int(get_response.mic_source)

    if actual_source != target_source_id:
        raise MicSourceError(
            "麦克风源确认失败："
            f"期望{target_source_id}，实际{actual_source}"
        )

    return actual_source


def ensure_external_mic(
    node,
    *,
    set_service_name: str,
    get_service_name: str,
    timeout_sec: float = 5.0,
    external_source_id: int = EXTERNAL_MIC_SOURCE_ID,
) -> int:
    """兼容上一版：切换并确认外置麦。"""
    return ensure_mic_source(
        node,
        target_source_id=external_source_id,
        set_service_name=set_service_name,
        get_service_name=get_service_name,
        timeout_sec=timeout_sec,
    )


def ensure_internal_mic(
    node,
    *,
    set_service_name: str,
    get_service_name: str,
    timeout_sec: float = 5.0,
) -> int:
    """切换并确认内置麦。"""
    return ensure_mic_source(
        node,
        target_source_id=INTERNAL_MIC_SOURCE_ID,
        set_service_name=set_service_name,
        get_service_name=get_service_name,
        timeout_sec=timeout_sec,
    )
