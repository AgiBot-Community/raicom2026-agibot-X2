#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

import numpy as np
import rclpy

from geometry_msgs.msg import PointStamped

from x2_ik_sdk import (
    ArmSide,
    X2ArmIKSolver,
    X2IKConfig,
)

from x2_safe_joint_microtest import (
    SafeJointMicrotest,
)


TARGET_TOPIC = "/vision/grasp_target_xyz"

TARGET_FRAME = "x2_ik_model"

SIDE = ArmSide.RIGHT


def fmt_xyz(v):
    return (
        "["
        + ", ".join(
            f"{float(x):+.6f}"
            for x in v
        )
        + "]"
    )


def main():

    parser = argparse.ArgumentParser(
        description=(
            "手动模拟视觉XYZ目标"
        )
    )

    group = (
        parser.add_mutually_exclusive_group(
            required=True
        )
    )

    group.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=(
            "X",
            "Y",
            "Z",
        ),
        help=(
            "直接发布绝对XYZ，单位m"
        ),
    )

    group.add_argument(
        "--offset",
        nargs=3,
        type=float,
        metavar=(
            "DX",
            "DY",
            "DZ",
        ),
        help=(
            "根据当前右手FK生成相对目标，单位m"
        ),
    )

    args = parser.parse_args()

    rclpy.init()

    node = SafeJointMicrotest()

    try:
        if args.target is not None:

            target_xyz = np.asarray(
                args.target,
                dtype=float,
            )

            print(
                "TARGET_SOURCE=MANUAL_ABSOLUTE"
            )

        else:
            solver = X2ArmIKSolver(
                X2IKConfig.default_omnipicker()
            )

            (
                current_arm,
                current_head,
                _,
            ) = node.read_state()

            current_xyz = np.asarray(
                solver.fk_xyz(
                    SIDE,
                    current_arm,
                    current_head_pos=(
                        current_head
                    ),
                ),
                dtype=float,
            )

            offset = np.asarray(
                args.offset,
                dtype=float,
            )

            target_xyz = (
                current_xyz
                + offset
            )

            print(
                "TARGET_SOURCE="
                "MANUAL_OFFSET_SIMULATED_VISION"
            )

            print(
                "CURRENT_XYZ="
                f"{fmt_xyz(current_xyz)}"
            )

            print(
                "OFFSET_M="
                f"{fmt_xyz(offset)}"
            )

        pub = node.create_publisher(
            PointStamped,
            TARGET_TOPIC,
            10,
        )

        # 等DDS发现subscriber
        deadline = (
            time.monotonic()
            + 3.0
        )

        while (
            rclpy.ok()
            and pub.get_subscription_count() < 1
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

        subscribers = (
            pub.get_subscription_count()
        )

        print(
            f"TARGET_SUBSCRIBERS={subscribers}"
        )

        if subscribers < 1:
            raise RuntimeError(
                "没有发现XYZ Executor Server订阅者"
            )

        msg = PointStamped()

        msg.header.stamp = (
            node.get_clock().now().to_msg()
        )

        msg.header.frame_id = (
            TARGET_FRAME
        )

        msg.point.x = float(
            target_xyz[0]
        )

        msg.point.y = float(
            target_xyz[1]
        )

        msg.point.z = float(
            target_xyz[2]
        )

        # 连发几帧只是确保测试阶段DDS收到；
        # Executor只缓存，不会因此运动。
        for _ in range(5):

            pub.publish(msg)

            rclpy.spin_once(
                node,
                timeout_sec=0.02,
            )

            time.sleep(0.05)

        print(
            f"TARGET_TOPIC={TARGET_TOPIC}"
        )

        print(
            f"TARGET_FRAME={TARGET_FRAME}"
        )

        print(
            "TARGET_XYZ="
            f"{fmt_xyz(target_xyz)}"
        )

        print(
            "REAL_ROBOT_MOTION=false"
        )

        print(
            "RESULT=PASS"
        )

        return 0

    except Exception as exc:

        print(
            "RESULT=FAIL"
        )

        print(
            f"REASON={exc}"
        )

        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
