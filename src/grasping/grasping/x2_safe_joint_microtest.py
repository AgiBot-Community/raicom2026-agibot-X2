#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import MessageHeader, UpperBodyCommandArray
from aimdk_msgs.srv import GetAllJointState

from x2_ik_sdk import X2ArmIKSolver, X2IKConfig
from x2_ik_sdk.config import ARM_POS_ORDER


SERVICE_NAME = "/aimdk_5Fmsgs/srv/GetAllJointState"
COMMAND_TOPIC = "/mc/upper_body_command"

HEAD_ORDER = [
    "head_yaw_joint",
    "head_pitch_joint",
]

SAFE_JOINTS = {
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
}

MAX_ALLOWED_DELTA = 0.05
MAX_STATIONARY_VELOCITY = 0.10
JOINT_LIMIT_MARGIN = 0.02
PUBLISH_RATE_HZ = 50.0


class SafeJointMicrotest(Node):
    def __init__(self) -> None:
        super().__init__("x2_safe_joint_microtest")

        self.state_client = self.create_client(
            GetAllJointState,
            SERVICE_NAME,
        )

        self.command_pub = self.create_publisher(
            UpperBodyCommandArray,
            COMMAND_TOPIC,
            10,
        )

        self.solver = X2ArmIKSolver(
            X2IKConfig.default_omnipicker()
        )

    def wait_for_command_subscriber(
        self,
        timeout_sec: float = 5.0,
    ) -> int:
        """等待ROS 2完成端点发现，返回订阅者数量。"""

        deadline = time.monotonic() + timeout_sec
        subscriber_count = 0

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

            subscriber_count = self.count_subscribers(
                COMMAND_TOPIC
            )

            if subscriber_count >= 1:
                return subscriber_count

        return subscriber_count

    def read_state(
        self,
        timeout_sec: float = 5.0,
    ) -> Tuple[List[float], List[float], Dict[str, float]]:
        if not self.state_client.wait_for_service(
            timeout_sec=timeout_sec
        ):
            raise RuntimeError(
                f"服务不可用：{SERVICE_NAME}"
            )

        request = GetAllJointState.Request()
        future = self.state_client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec,
        )

        if not future.done():
            raise RuntimeError(
                "GetAllJointState调用超时"
            )

        response = future.result()
        if response is None:
            raise RuntimeError(
                "GetAllJointState返回空响应"
            )

        arm_items = list(response.arm_joints)
        head_items = list(response.head_joints)

        arm_by_name = {
            item.name: item
            for item in arm_items
        }
        head_by_name = {
            item.name: item
            for item in head_items
        }

        missing_arm = [
            name
            for name in ARM_POS_ORDER
            if name not in arm_by_name
        ]
        missing_head = [
            name
            for name in HEAD_ORDER
            if name not in head_by_name
        ]

        if missing_arm:
            raise RuntimeError(
                f"缺少手臂关节：{missing_arm}"
            )

        if missing_head:
            raise RuntimeError(
                f"缺少头部关节：{missing_head}"
            )

        bad_error_codes = [
            item.name
            for item in arm_items + head_items
            if int(item.error_code) != 0
        ]

        if bad_error_codes:
            raise RuntimeError(
                "关节error_code非零："
                f"{bad_error_codes}"
            )

        moving_joints = {
            item.name: float(item.velocity)
            for item in arm_items
            if abs(float(item.velocity))
            > MAX_STATIONARY_VELOCITY
        }

        if moving_joints:
            raise RuntimeError(
                "机器人手臂当前仍在运动："
                f"{moving_joints}"
            )

        arm_pos = [
            float(arm_by_name[name].position)
            for name in ARM_POS_ORDER
        ]

        head_pos = [
            float(head_by_name[name].position)
            for name in HEAD_ORDER
        ]

        velocity_by_name = {
            item.name: float(item.velocity)
            for item in arm_items
        }

        return arm_pos, head_pos, velocity_by_name

    def get_limit_map(
        self,
    ) -> Dict[str, Tuple[float, float]]:
        return {
            name: (lower, upper)
            for name, lower, upper
            in self.solver.joint_limits_for_arm_pos()
        }

    def validate_target(
        self,
        current_arm: List[float],
        joint_name: str,
        delta: float,
    ) -> Tuple[List[float], int]:
        if joint_name not in SAFE_JOINTS:
            raise RuntimeError(
                f"首次测试只允许以下腕部关节："
                f"{sorted(SAFE_JOINTS)}"
            )

        if not math.isfinite(delta):
            raise RuntimeError("delta不是有限数值")

        if abs(delta) > MAX_ALLOWED_DELTA:
            raise RuntimeError(
                f"delta={delta:.6f}超过首次测试上限"
                f"{MAX_ALLOWED_DELTA:.6f} rad"
            )

        index = ARM_POS_ORDER.index(joint_name)
        target_arm = list(current_arm)
        target_arm[index] += delta

        limit_map = self.get_limit_map()
        lower, upper = limit_map[joint_name]
        target = target_arm[index]

        if target <= lower + JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                f"目标过于接近下限：target={target:.6f}, "
                f"lower={lower:.6f}"
            )

        if target >= upper - JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                f"目标过于接近上限：target={target:.6f}, "
                f"upper={upper:.6f}"
            )

        for i, (current, goal) in enumerate(
            zip(current_arm, target_arm)
        ):
            actual_delta = goal - current

            if i == index:
                if abs(actual_delta - delta) > 1e-9:
                    raise RuntimeError(
                        "目标关节变化量校验失败"
                    )
            elif abs(actual_delta) > 1e-12:
                raise RuntimeError(
                    f"非目标关节发生变化："
                    f"{ARM_POS_ORDER[i]}"
                )

        return target_arm, index

    def publish_target(
        self,
        target_arm: List[float],
        current_head: List[float],
        seconds: float,
    ) -> int:
        subscriber_count = (
            self.wait_for_command_subscriber(
                timeout_sec=5.0,
            )
        )

        if subscriber_count < 1:
            raise RuntimeError(
                f"{COMMAND_TOPIC}在5秒内未发现订阅者"
            )

        # 创建发布者后，本节点自身通常计入发布者数量。
        publisher_count = self.count_publishers(
            COMMAND_TOPIC
        )

        if publisher_count > 1:
            raise RuntimeError(
                f"{COMMAND_TOPIC}存在其他发布者："
                f"publisher_count={publisher_count}"
            )

        period = 1.0 / PUBLISH_RATE_HZ
        deadline = time.monotonic() + seconds
        sequence = 0

        while (
            time.monotonic() < deadline
            and rclpy.ok()
        ):
            message = UpperBodyCommandArray()

            now = self.get_clock().now()
            message.header = MessageHeader()
            message.header.stamp.sec = (
                now.nanoseconds // 1_000_000_000
            )
            message.header.stamp.nanosec = (
                now.nanoseconds % 1_000_000_000
            )
            message.header.frame_id = "mc_upper_body"
            message.header.sequence = sequence

            message.source = "upper_body_example"

            # 0表示本次不发送手部动作。
            message.hand_sub_mode = 0
            message.hand_pos = []

            # 保持当前头部关节角，不发送零位目标。
            message.head_pos = [
                float(value)
                for value in current_head
            ]

            # 完整14维：只有选定腕部关节发生微小变化。
            message.arm_pos = [
                float(value)
                for value in target_arm
            ]

            self.command_pub.publish(message)
            rclpy.spin_once(
                self,
                timeout_sec=0.001,
            )

            sequence += 1
            time.sleep(period)

        return sequence


def main() -> int:
    parser = argparse.ArgumentParser(
        description="X2首次真机腕部微动作安全测试"
    )

    parser.add_argument(
        "--joint",
        default="right_wrist_roll_joint",
        choices=sorted(SAFE_JOINTS),
    )

    parser.add_argument(
        "--delta",
        type=float,
        default=0.03,
        help="关节变化量，单位rad，绝对值不得超过0.05",
    )

    parser.add_argument(
        "--publish-seconds",
        type=float,
        default=2.0,
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="显式允许真实发布；不提供时只进行dry-run",
    )

    args = parser.parse_args()

    if args.publish_seconds <= 0.0:
        print("RESULT=FAIL")
        print("REASON=publish-seconds必须大于0")
        return 2

    if args.publish_seconds > 3.0:
        print("RESULT=FAIL")
        print("REASON=首次测试发布时长不得超过3秒")
        return 2

    rclpy.init()
    node = SafeJointMicrotest()

    try:
        current_arm, current_head, velocities = (
            node.read_state()
        )

        target_arm, target_index = (
            node.validate_target(
                current_arm=current_arm,
                joint_name=args.joint,
                delta=args.delta,
            )
        )

        current_value = current_arm[target_index]
        target_value = target_arm[target_index]

        subscriber_count = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0,
            )
        )

        print("=== X2首次腕部微动作检查 ===")
        print(f"COMMAND_TOPIC={COMMAND_TOPIC}")
        print(
            "COMMAND_SUBSCRIBERS="
            f"{subscriber_count}"
        )
        print(f"JOINT={args.joint}")
        print(f"CURRENT={current_value:.9f}")
        print(f"TARGET={target_value:.9f}")
        print(f"DELTA={args.delta:+.9f}")
        print(
            "DELTA_DEG="
            f"{math.degrees(args.delta):+.6f}"
        )
        print(
            "HEAD_CURRENT="
            + ",".join(
                f"{value:.9f}"
                for value in current_head
            )
        )
        print(
            "MAX_ARM_VELOCITY="
            f"{max(abs(v) for v in velocities.values()):.9f}"
        )
        print("HAND_CONTROL=DISABLED")
        print("OTHER_ARM_JOINTS=UNCHANGED")

        if not args.execute:
            print("DRY_RUN=true")
            print("REAL_COMMAND_PUBLISHED=false")
            print("RESULT=PASS")
            return 0

        print("DRY_RUN=false")
        print("PUBLISH_ATTEMPTED=true")

        published = node.publish_target(
            target_arm=target_arm,
            current_head=current_head,
            seconds=args.publish_seconds,
        )

        print(f"PUBLISHED_FRAMES={published}")
        print("REAL_COMMAND_PUBLISHED=true")

        time.sleep(1.0)

        after_arm, after_head, _ = node.read_state()
        observed = (
            after_arm[target_index]
            - current_arm[target_index]
        )

        print(
            "OBSERVED_JOINT_POSITION="
            f"{after_arm[target_index]:.9f}"
        )
        print(
            "OBSERVED_DELTA="
            f"{observed:+.9f}"
        )
        print(
            "OBSERVED_HEAD="
            + ",".join(
                f"{value:.9f}"
                for value in after_head
            )
        )
        # 不能只根据“成功发布”判定动作成功，
        # 必须确认真机关节状态产生了同方向可测变化。
        if abs(observed) < 0.005:
            print("RESULT=FAIL")
            print(
                "REASON=控制帧已发布，但关节未产生可测位移"
            )
            return 1

        if observed * args.delta <= 0.0:
            print("RESULT=FAIL")
            print(
                "REASON=关节实际运动方向与目标方向不一致"
            )
            return 1

        if abs(observed) > abs(args.delta) + 0.03:
            print("RESULT=FAIL")
            print(
                "REASON=关节实际变化明显超过目标变化量"
            )
            return 1

        print("RESULT=PASS")
        return 0

    except Exception as exc:
        print("RESULT=FAIL")
        print(f"REASON={exc}")
        return 1

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
