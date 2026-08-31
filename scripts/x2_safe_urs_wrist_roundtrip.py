#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time

import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from aimdk_msgs.msg import (
    CommonState,
    JointCommandArray,
    McActionCommand,
    MessageHeader,
    RequestHeader,
    UpperBodyCommandArray,
)
from aimdk_msgs.srv import SetMcAction

from x2_safe_joint_microtest import (
    ARM_POS_ORDER,
    COMMAND_TOPIC,
    SafeJointMicrotest,
)


MODE_SERVICE = "/aimdk_5Fmsgs/srv/SetMcAction"
HAL_ARM_TOPIC = "/aima/hal/joint/arm/command"

JOINT_NAME = "right_wrist_roll_joint"

DELTA_RAD = math.radians(10.0)
RAMP_SECONDS = 5.0
HOLD_SECONDS = 1.5
PRE_SWITCH_HOLD_SECONDS = 2.0
POST_SWITCH_HOLD_SECONDS = 2.0

PUBLISH_RATE_HZ = 50.0
JOINT_LIMIT_MARGIN = 0.03


class SafeUrsWristTest(SafeJointMicrotest):
    def __init__(self) -> None:
        super().__init__()

        self.mode_client = self.create_client(
            SetMcAction,
            MODE_SERVICE,
        )

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.hal_wrist_values = []

        self.create_subscription(
            JointCommandArray,
            HAL_ARM_TOPIC,
            self._on_hal_arm_command,
            qos,
        )

        self.sequence = 0

    def _on_hal_arm_command(
        self,
        message: JointCommandArray,
    ) -> None:
        for joint in message.joints:
            if joint.name == JOINT_NAME:
                self.hal_wrist_values.append(
                    float(joint.position)
                )
                return

    def publish_once(
        self,
        arm_position,
        head_position,
    ) -> None:
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
        message.header.sequence = self.sequence

        self.sequence += 1

        # 与官方upper_body_control示例保持一致。
        message.source = "upper_body_example"

        # 本测试不控制手部。
        message.hand_sub_mode = 0
        message.hand_pos = []

        # 保持真实头部位置。
        message.head_pos = [
            float(value)
            for value in head_position
        ]

        # 始终发送完整14维手臂关节位置。
        message.arm_pos = [
            float(value)
            for value in arm_position
        ]

        self.command_pub.publish(message)

    def publish_segment(
        self,
        start_arm,
        target_arm,
        head_position,
        duration_sec: float,
    ) -> int:
        if duration_sec <= 0.0:
            return 0

        period = 1.0 / PUBLISH_RATE_HZ
        start_time = time.monotonic()
        deadline = start_time + duration_sec
        next_tick = start_time
        frames = 0

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            now = time.monotonic()
            elapsed = now - start_time

            alpha = min(
                1.0,
                max(0.0, elapsed / duration_sec),
            )

            command_arm = [
                float(start + (goal - start) * alpha)
                for start, goal in zip(
                    start_arm,
                    target_arm,
                )
            ]

            self.publish_once(
                arm_position=command_arm,
                head_position=head_position,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.001,
            )

            frames += 1
            next_tick += period

            sleep_seconds = next_tick - time.monotonic()

            if sleep_seconds > 0.0:
                time.sleep(sleep_seconds)

        return frames

    def set_mode_while_holding(
        self,
        action_name: str,
        hold_arm,
        hold_head,
        timeout_sec: float = 5.0,
    ) -> None:
        if not self.mode_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"模式服务不可用：{MODE_SERVICE}"
            )

        request = SetMcAction.Request()
        request.header = RequestHeader()
        request.header.stamp = (
            self.get_clock().now().to_msg()
        )
        request.source = "node.set_mc_action"

        command = McActionCommand()
        command.action_desc = action_name
        request.command = command

        print(f"MODE_REQUEST={action_name}")

        future = self.mode_client.call_async(request)
        deadline = time.monotonic() + timeout_sec

        period = 1.0 / PUBLISH_RATE_HZ
        next_tick = time.monotonic()

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            # 模式切换期间持续发送当前安全姿态。
            self.publish_once(
                arm_position=hold_arm,
                head_position=hold_head,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.005,
            )

            next_tick += period
            sleep_seconds = next_tick - time.monotonic()

            if sleep_seconds > 0.0:
                time.sleep(sleep_seconds)

        if not future.done():
            raise RuntimeError(
                f"切换模式超时：{action_name}"
            )

        try:
            response = future.result()
        except Exception as exc:
            raise RuntimeError(
                f"切换模式调用异常：{exc}"
            ) from exc

        if response is None:
            raise RuntimeError(
                f"切换模式返回空响应：{action_name}"
            )

        status = int(response.response.status.value)
        message = str(response.response.message)

        print(f"MODE_RESPONSE_STATUS={status}")
        print(f"MODE_RESPONSE_MESSAGE={message!r}")

        if status != int(CommonState.SUCCESS):
            raise RuntimeError(
                f"模式切换失败：{action_name}，"
                f"status={status}，message={message!r}"
            )

        print(f"MODE_SET_SUCCESS={action_name}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="X2 URS模式右腕10度安全往返测试"
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="显式允许切换URS并执行真实动作",
    )

    args = parser.parse_args()

    rclpy.init()
    node = SafeUrsWristTest()

    switched_to_urs = False
    current_arm = None
    current_head = None

    try:
        current_arm, current_head, velocities = (
            node.read_state()
        )

        subscribers = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0,
            )
        )

        if subscribers < 1:
            raise RuntimeError(
                f"{COMMAND_TOPIC}没有订阅者"
            )

        publisher_count = node.count_publishers(
            COMMAND_TOPIC
        )

        # 本节点自身通常计入1个publisher。
        if publisher_count > 1:
            raise RuntimeError(
                f"{COMMAND_TOPIC}存在其他发布者："
                f"publisher_count={publisher_count}"
            )

        joint_index = ARM_POS_ORDER.index(
            JOINT_NAME
        )

        current_value = float(
            current_arm[joint_index]
        )
        target_value = current_value + DELTA_RAD

        limit_map = node.get_limit_map()
        lower, upper = limit_map[JOINT_NAME]

        if target_value <= lower + JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                "目标过于接近关节下限："
                f"target={target_value:.6f}，"
                f"lower={lower:.6f}"
            )

        if target_value >= upper - JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                "目标过于接近关节上限："
                f"target={target_value:.6f}，"
                f"upper={upper:.6f}"
            )

        target_arm = list(current_arm)
        target_arm[joint_index] = target_value

        print("=== X2 URS右腕10度安全往返测试 ===")
        print(f"COMMAND_SUBSCRIBERS={subscribers}")
        print(f"COMMAND_PUBLISHERS={publisher_count}")
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
        print("OTHER_ARM_JOINTS=UNCHANGED")
        print("HAND_CONTROL=DISABLED")
        print(
            f"RAMP_SECONDS={RAMP_SECONDS:.1f}"
        )
        print("RETURN_TO_START=true")
        print("FINAL_MODE=STAND_DEFAULT")

        if not args.execute:
            print("DRY_RUN=true")
            print("MODE_CHANGED=false")
            print("REAL_COMMAND_PUBLISHED=false")
            print("RESULT=PASS")
            return 0

        print("DRY_RUN=false")
        print("REAL_TEST_STARTED=true")

        # 1. 在原模式下预先持续发布当前姿态。
        pre_frames = node.publish_segment(
            start_arm=current_arm,
            target_arm=current_arm,
            head_position=current_head,
            duration_sec=PRE_SWITCH_HOLD_SECONDS,
        )
        print(f"PRE_SWITCH_HOLD_FRAMES={pre_frames}")

        # 2. 发布当前姿态期间切换到URS。
        node.set_mode_while_holding(
            action_name="UPPERBODY_REMOTE_SPLIT",
            hold_arm=current_arm,
            hold_head=current_head,
        )
        switched_to_urs = True

        # 3. 切换后继续保持当前真实姿态。
        post_frames = node.publish_segment(
            start_arm=current_arm,
            target_arm=current_arm,
            head_position=current_head,
            duration_sec=POST_SWITCH_HOLD_SECONDS,
        )
        print(f"POST_SWITCH_HOLD_FRAMES={post_frames}")

        node.hal_wrist_values.clear()

        # 4. 右腕缓慢转动10度。
        outbound_frames = node.publish_segment(
            start_arm=current_arm,
            target_arm=target_arm,
            head_position=current_head,
            duration_sec=RAMP_SECONDS,
        )
        print(f"OUTBOUND_FRAMES={outbound_frames}")

        peak_hold_frames = node.publish_segment(
            start_arm=target_arm,
            target_arm=target_arm,
            head_position=current_head,
            duration_sec=HOLD_SECONDS,
        )
        print(f"PEAK_HOLD_FRAMES={peak_hold_frames}")

        peak_arm, peak_head, _ = node.read_state()
        peak_position = float(
            peak_arm[joint_index]
        )
        peak_delta = peak_position - current_value

        print(
            "PEAK_OBSERVED_POSITION="
            f"{peak_position:.9f}"
        )
        print(
            "PEAK_OBSERVED_DELTA_RAD="
            f"{peak_delta:+.9f}"
        )
        print(
            "PEAK_OBSERVED_DELTA_DEG="
            f"{math.degrees(peak_delta):+.6f}"
        )
        print(
            "PEAK_OBSERVED_HEAD="
            + ",".join(
                f"{value:.9f}"
                for value in peak_head
            )
        )

        # 5. 无论峰值是否达到目标，都缓慢返回起始命令。
        return_frames = node.publish_segment(
            start_arm=target_arm,
            target_arm=current_arm,
            head_position=current_head,
            duration_sec=RAMP_SECONDS,
        )
        print(f"RETURN_FRAMES={return_frames}")

        return_hold_frames = node.publish_segment(
            start_arm=current_arm,
            target_arm=current_arm,
            head_position=current_head,
            duration_sec=HOLD_SECONDS,
        )
        print(
            "RETURN_HOLD_FRAMES="
            f"{return_hold_frames}"
        )

        return_arm, return_head, _ = node.read_state()
        return_position = float(
            return_arm[joint_index]
        )
        return_delta = (
            return_position - current_value
        )

        print(
            "RETURN_OBSERVED_POSITION="
            f"{return_position:.9f}"
        )
        print(
            "RETURN_OBSERVED_DELTA_RAD="
            f"{return_delta:+.9f}"
        )
        print(
            "RETURN_OBSERVED_DELTA_DEG="
            f"{math.degrees(return_delta):+.6f}"
        )

        # 6. 保持起始姿态时切回稳定站立。
        node.set_mode_while_holding(
            action_name="STAND_DEFAULT",
            hold_arm=current_arm,
            hold_head=current_head,
        )
        switched_to_urs = False

        final_frames = node.publish_segment(
            start_arm=current_arm,
            target_arm=current_arm,
            head_position=current_head,
            duration_sec=1.0,
        )
        print(f"FINAL_HOLD_FRAMES={final_frames}")

        if node.hal_wrist_values:
            print(
                "HAL_WRIST_MIN="
                f"{min(node.hal_wrist_values):.9f}"
            )
            print(
                "HAL_WRIST_MAX="
                f"{max(node.hal_wrist_values):.9f}"
            )
            print(
                "HAL_WRIST_RANGE="
                f"{max(node.hal_wrist_values) - min(node.hal_wrist_values):.9f}"
            )
        else:
            print("HAL_WRIST_DATA=NONE")

        if abs(peak_delta) < math.radians(2.0):
            print("RESULT=FAIL")
            print(
                "REASON=URS模式下实际运动仍小于2度"
            )
            return 1

        if peak_delta * DELTA_RAD <= 0.0:
            print("RESULT=FAIL")
            print(
                "REASON=实际运动方向与目标方向不一致"
            )
            return 1

        if abs(peak_delta) > math.radians(13.0):
            print("RESULT=FAIL")
            print(
                "REASON=实际运动明显超过10度目标"
            )
            return 1

        if abs(return_delta) > math.radians(3.0):
            print("RESULT=FAIL")
            print(
                "REASON=右腕未返回起始位置附近"
            )
            return 1

        print("RESULT=PASS")
        return 0

    except Exception as exc:
        print("RESULT=FAIL")
        print(f"REASON={exc}")
        return 1

    finally:
        if (
            switched_to_urs
            and current_arm is not None
            and current_head is not None
        ):
            print(
                "RECOVERY=尝试保持起始姿态并切回STAND_DEFAULT"
            )

            try:
                node.publish_segment(
                    start_arm=current_arm,
                    target_arm=current_arm,
                    head_position=current_head,
                    duration_sec=1.0,
                )

                node.set_mode_while_holding(
                    action_name="STAND_DEFAULT",
                    hold_arm=current_arm,
                    hold_head=current_head,
                )

                print("RECOVERY_RESULT=PASS")

            except Exception as recovery_exc:
                print("RECOVERY_RESULT=FAIL")
                print(
                    f"RECOVERY_REASON={recovery_exc}"
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
