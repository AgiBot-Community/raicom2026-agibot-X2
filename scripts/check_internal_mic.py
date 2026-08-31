#!/usr/bin/env python3

import time
from typing import Any

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import (
    GetMicSourceRequest,
    SetMicSourceRequest,
)


GET_SERVICE = "/aimdk_5Fmsgs/srv/GetMicSourceRequest"
SET_SERVICE = "/aimdk_5Fmsgs/srv/SetMicSourceRequest"


def call_service(
    node: Node,
    client: Any,
    request: Any,
    name: str,
    attempts: int = 6,
) -> Any | None:
    for attempt in range(1, attempts + 1):
        request.header.header.stamp = (
            node.get_clock().now().to_msg()
        )

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=1.0,
        )

        if future.done():
            try:
                response = future.result()
            except Exception as exc:
                print(
                    f"{name}第{attempt}次调用异常：{exc}"
                )
            else:
                if response is not None:
                    return response

        print(
            f"{name}第{attempt}次未成功，继续重试……"
        )
        time.sleep(0.2)

    return None


def query_source(
    node: Node,
    client: Any,
) -> int | None:
    request = GetMicSourceRequest.Request()

    response = call_service(
        node,
        client,
        request,
        "查询麦克风来源",
    )

    if response is None:
        return None

    status = int(response.header.status.value)
    source = int(response.mic_source)

    print(
        f"查询结果：status={status}, "
        f"mic_source={source}, "
        f"message={response.header.message!r}"
    )

    if status != 1:
        return None

    return source


def main() -> None:
    rclpy.init()
    node = Node("check_internal_mic")

    get_client = node.create_client(
        GetMicSourceRequest,
        GET_SERVICE,
    )

    set_client = node.create_client(
        SetMicSourceRequest,
        SET_SERVICE,
    )

    try:
        print("等待麦克风控制服务……")

        if not get_client.wait_for_service(
            timeout_sec=8.0
        ):
            raise RuntimeError(
                f"服务不可用：{GET_SERVICE}"
            )

        if not set_client.wait_for_service(
            timeout_sec=8.0
        ):
            raise RuntimeError(
                f"服务不可用：{SET_SERVICE}"
            )

        current_source = query_source(
            node,
            get_client,
        )

        if current_source is None:
            raise RuntimeError(
                "无法确认当前麦克风来源"
            )

        if current_source == 0:
            print(
                "当前已经是内置麦克风，"
                "不需要重新设置。"
            )
            return

        print(
            f"当前mic_source={current_source}，"
            "现在恢复为内置麦克风0。"
        )

        set_request = SetMicSourceRequest.Request()
        set_request.mic_source = 0

        set_response = call_service(
            node,
            set_client,
            set_request,
            "设置内置麦克风",
        )

        if set_response is None:
            raise RuntimeError(
                "设置内置麦克风没有响应"
            )

        print(
            "设置结果："
            f"status={int(set_response.header.status.value)}, "
            f"message={set_response.header.message!r}"
        )

        if int(set_response.header.status.value) != 1:
            raise RuntimeError(
                "hal_audio未接受内置麦克风设置"
            )

        time.sleep(3.0)

        confirmed_source = query_source(
            node,
            get_client,
        )

        if confirmed_source != 0:
            raise RuntimeError(
                "重新查询后仍不是内置麦克风"
            )

        print("内置麦克风已恢复。")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
