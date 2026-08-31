#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class ListenTrigger(Node):

    def __init__(self) -> None:
        super().__init__("listen_trigger_once")

        self.publisher = self.create_publisher(
            Bool,
            "/ai_agent/listen_control",
            10,
        )


def main() -> None:
    rclpy.init()

    node = ListenTrigger()

    expected_subscribers = 2
    timeout_seconds = 10.0
    deadline = time.monotonic() + timeout_seconds

    print(
        "等待 /ai_agent/listen_control "
        f"的 {expected_subscribers} 个订阅者……"
    )

    while (
        node.publisher.get_subscription_count()
        < expected_subscribers
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

        if time.monotonic() >= deadline:
            actual = (
                node.publisher
                .get_subscription_count()
            )

            print(
                "等待超时，当前订阅者数量：",
                actual,
            )

            node.destroy_node()
            rclpy.shutdown()
            raise SystemExit(1)

    actual = (
        node.publisher
        .get_subscription_count()
    )

    print(
        f"已匹配 {actual} 个订阅者，"
        "发布 listen_control=true"
    )

    message = Bool()
    message.data = True

    node.publisher.publish(message)

    # 保留发布者一段时间，
    # 确保DDS有时间完成数据发送。
    for _ in range(10):
        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

    print("发布完成")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
