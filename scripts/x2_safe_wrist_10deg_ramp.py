#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time

import rclpy

from aimdk_msgs.msg import MessageHeader, UpperBodyCommandArray

from x2_safe_joint_microtest import (
    ARM_POS_ORDER,
    COMMAND_TOPIC,
    SafeJointMicrotest,
)


JOINT_NAME = "right_wrist_roll_joint"
DELTA_RAD = math.radians(10.0)

RAMP_SECONDS = 5.0
HOLD_SECONDS = 1.0
PUBLISH_RATE_HZ = 50.0
JOINT_LIMIT_MARGIN = 0.03


class WristRampTest(SafeJointMicrotest):
    def publish_ramp(
        self,
        current_arm,
        target_arm,
        current_head,
    ):
        subscriber_count = self.wait_for_command_subscriber(
            timeout_sec=5.0
        )

        if subscriber_count < 1:
            raise RuntimeError(
                f"{COMMAND_TOPIC}在5秒内未发现订阅者"
            )

        # 本节点自身会占用一个publisher。
        publisher_count = self.count_publishers(
            COMMAND_TOPIC
        )

        if publisher_count > 1:
            raise RuntimeError(
                f"{COMMAND_TOPIC}存在其他发布者："
                f"publisher_count={publisher_count}"
            )

        period = 1.0 / PUBLISH_RATE_HZ
        total_seconds = RAMP_SECONDS + HOLD_SECONDS

        start_time = time.monotonic()
        deadline = start_time + total_seconds
        sequence = 0

        while rclpy.ok() and time.monotonic() < deadline:
            elapsed = time.monotonic() - start_time

            # 前5秒线性运动，最后1秒保持最终目标。
            alpha = min(
                1.0,
                max(0.0, elapsed / RAMP_SECONDS),
            )

            command_arm = [
                float(start + (goal - start) * alpha)
                for start, goal in zip(
                    current_arm,
                    target_arm,
                )
            ]

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

            # 使用官方示例来源标识。
            message.source = "upper_body_example"

            # 不控制夹爪。
            message.hand_sub_mode = 0
            message.hand_pos = []

            # 保持真实头部位置。
            message.head_pos = [
                float(value)
                for value in current_head
            ]

            message.arm_pos = command_arm

            self.command_pub.publish(message)

            rclpy.spin_once(
                self,
                timeout_sec=0.001,
            )

            sequence += 1
            time.sleep(period)

        return sequence


def main():
    parser = argparse.ArgumentParser(
        description="X2右腕10度渐变真机测试"
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="提供该参数后才发布真机指令",
    )

    args = parser.parse_args()

    rclpy.init()
    node = WristRampTest()

    try:
        current_arm, current_head, velocities = (
            node.read_state()
        )

        joint_index = ARM_POS_ORDER.index(
            JOINT_NAME
        )

        current_value = current_arm[joint_index]
        target_value = current_value + DELTA_RAD

        limit_map = node.get_limit_map()
        lower, upper = limit_map[JOINT_NAME]

        if target_value <= lower + JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                f"目标过于接近关节下限："
                f"target={target_value:.6f}, "
                f"lower={lower:.6f}"
            )

        if target_value >= upper - JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                f"目标过于接近关节上限："
                f"target={target_value:.6f}, "
                f"upper={upper:.6f}"
            )

        target_arm = list(current_arm)
        target_arm[joint_index] = target_value

        subscribers = node.wait_for_command_subscriber(
            timeout_sec=5.0
        )

        print("=== X2右腕10度渐变测试 ===")
        print(f"COMMAND_TOPIC={COMMAND_TOPIC}")
        print(f"COMMAND_SUBSCRIBERS={subscribers}")
        print(f"JOINT={JOINT_NAME}")
        print(f"CURRENT={current_value:.9f}")
        print(f"TARGET={target_value:.9f}")
        print(f"DELTA_RAD={DELTA_RAD:+.9f}")
        print("DELTA_DEG=+10.000000")
        print(f"JOINT_LOWER={lower:.9f}")
        print(f"JOINT_UPPER={upper:.9f}")
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
        print(f"RAMP_SECONDS={RAMP_SECONDS:.1f}")
        print(f"HOLD_SECONDS={HOLD_SECONDS:.1f}")
        print("HAND_CONTROL=DISABLED")
        print("OTHER_ARM_JOINTS=UNCHANGED")

        if not args.execute:
            print("DRY_RUN=true")
            print("REAL_COMMAND_PUBLISHED=false")
            print("RESULT=PASS")
            return 0

        print("DRY_RUN=false")
        print("PUBLISH_ATTEMPTED=true")

        published = node.publish_ramp(
            current_arm=current_arm,
            target_arm=target_arm,
            current_head=current_head,
        )

        print(f"PUBLISHED_FRAMES={published}")
        print("REAL_COMMAND_PUBLISHED=true")

        time.sleep(1.0)

        after_arm, after_head, _ = node.read_state()

        observed_position = after_arm[joint_index]
        observed_delta = (
            observed_position - current_value
        )

        print(
            "OBSERVED_JOINT_POSITION="
            f"{observed_position:.9f}"
        )
        print(
            "OBSERVED_DELTA_RAD="
            f"{observed_delta:+.9f}"
        )
        print(
            "OBSERVED_DELTA_DEG="
            f"{math.degrees(observed_delta):+.6f}"
        )
        print(
            "OBSERVED_HEAD="
            + ",".join(
                f"{value:.9f}"
                for value in after_head
            )
        )

        if abs(observed_delta) < math.radians(2.0):
            print("RESULT=FAIL")
            print(
                "REASON=发布10度目标后，实际变化仍小于2度"
            )
            return 1

        if observed_delta * DELTA_RAD <= 0.0:
            print("RESULT=FAIL")
            print(
                "REASON=实际运动方向与指令方向不一致"
            )
            return 1

        if abs(observed_delta) > math.radians(13.0):
            print("RESULT=FAIL")
            print(
                "REASON=实际变化明显超过10度目标"
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
