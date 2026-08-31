#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import time

import rclpy

from rclpy.node import Node
from std_msgs.msg import String


TOPIC = "/competition/grasp_target"


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--object",
        default="1",
    )

    parser.add_argument(
        "x",
        type=float,
    )

    parser.add_argument(
        "y",
        type=float,
    )

    parser.add_argument(
        "z",
        type=float,
    )

    args = parser.parse_args()

    rclpy.init()

    node = Node(
        "manual_grasp_target_sender"
    )

    pub = node.create_publisher(
        String,
        TOPIC,
        10,
    )

    deadline = (
        time.monotonic()
        + 3.0
    )

    while (
        pub.get_subscription_count() < 1
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

    if (
        pub.get_subscription_count()
        < 1
    ):
        print(
            "RESULT=FAIL"
        )

        print(
            "REASON=no subscriber"
        )

        return

    payload = {
        "object_name": args.object,
        "target_point": [
            args.x,
            args.y,
            args.z,
        ],
    }

    msg = String()

    msg.data = json.dumps(
        payload,
        ensure_ascii=False,
    )

    for _ in range(5):

        pub.publish(msg)

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

        time.sleep(0.05)

    print(
        "TOPIC="
        f"{TOPIC}"
    )

    print(
        "JSON="
        f"{msg.data}"
    )

    print(
        "REAL_ROBOT_MOTION=false"
    )

    print(
        "RESULT=PASS"
    )

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
