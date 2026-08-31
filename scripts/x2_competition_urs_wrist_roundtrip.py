#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import time

import rclpy
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
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

PUBLISH_RATE_HZ = 50.0

PRE_URS_HOLD_SECONDS = 2.0
POST_URS_HOLD_SECONDS = 2.0

RAMP_SECONDS = 5.0
PEAK_HOLD_SECONDS = 1.5
RETURN_HOLD_SECONDS = 1.5

JOINT_LIMIT_MARGIN = 0.03

# URS接管后，HAL目标与当前真实位置最大允许偏差。
TAKEOVER_MAX_ERROR = 0.03


class CompetitionUrsWristRoundtrip(
    SafeJointMicrotest
):

    def __init__(self):
        super().__init__()

        self.mode_client = self.create_client(
            SetMcAction,
            MODE_SERVICE,
        )

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.hal_wrist_values = []

        self.create_subscription(
            JointCommandArray,
            HAL_ARM_TOPIC,
            self.on_hal,
            qos,
        )

        self.sequence = 0

    def on_hal(self, msg):
        for joint in msg.joints:
            if joint.name == JOINT_NAME:
                self.hal_wrist_values.append(
                    float(joint.position)
                )
                return

    def publish_upper(
        self,
        arm_pos,
        head_pos,
        hand_pos,
    ):
        msg = UpperBodyCommandArray()

        now = self.get_clock().now()

        msg.header = MessageHeader()
        msg.header.stamp.sec = (
            now.nanoseconds // 1_000_000_000
        )
        msg.header.stamp.nanosec = (
            now.nanoseconds % 1_000_000_000
        )
        msg.header.frame_id = "mc_upper_body"
        msg.header.sequence = self.sequence

        self.sequence += 1

        msg.source = "upper_body_example"

        # 比赛机器人左右均为CLAW。
        msg.hand_sub_mode = (
            UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE
        )

        # 当前两只夹爪均为0.0。
        msg.hand_pos = [
            float(x)
            for x in hand_pos
        ]

        msg.head_pos = [
            float(x)
            for x in head_pos
        ]

        msg.arm_pos = [
            float(x)
            for x in arm_pos
        ]

        self.command_pub.publish(msg)

    def publish_segment(
        self,
        start_arm,
        target_arm,
        head_pos,
        hand_pos,
        duration,
    ):
        period = 1.0 / PUBLISH_RATE_HZ

        start_time = time.monotonic()
        deadline = start_time + duration
        next_tick = start_time

        frames = 0

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            elapsed = (
                time.monotonic() - start_time
            )

            alpha = min(
                1.0,
                max(
                    0.0,
                    elapsed / duration,
                ),
            )

            command_arm = [
                float(
                    q0 + (q1 - q0) * alpha
                )
                for q0, q1 in zip(
                    start_arm,
                    target_arm,
                )
            ]

            self.publish_upper(
                command_arm,
                head_pos,
                hand_pos,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.001,
            )

            frames += 1

            next_tick += period

            remaining = (
                next_tick - time.monotonic()
            )

            if remaining > 0.0:
                time.sleep(remaining)

        return frames

    def set_mode_while_holding(
        self,
        action_name,
        arm_pos,
        head_pos,
        hand_pos,
    ):
        if not self.mode_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "SetMcAction服务不可用"
            )

        req = SetMcAction.Request()

        req.header = RequestHeader()
        req.header.stamp = (
            self.get_clock().now().to_msg()
        )

        req.source = "node.set_mc_action"

        cmd = McActionCommand()
        cmd.action_desc = action_name

        req.command = cmd

        print(
            f"MODE_REQUEST={action_name}"
        )

        future = (
            self.mode_client.call_async(req)
        )

        deadline = (
            time.monotonic() + 5.0
        )

        period = 1.0 / PUBLISH_RATE_HZ
        next_tick = time.monotonic()

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            self.publish_upper(
                arm_pos,
                head_pos,
                hand_pos,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.005,
            )

            next_tick += period

            remaining = (
                next_tick - time.monotonic()
            )

            if remaining > 0:
                time.sleep(remaining)

        if not future.done():
            raise RuntimeError(
                f"模式切换超时：{action_name}"
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                f"模式切换空响应：{action_name}"
            )

        status = int(
            response.response.status.value
        )

        print(
            f"MODE_RESPONSE_STATUS={status}"
        )
        print(
            "MODE_RESPONSE_MESSAGE="
            f"{response.response.message!r}"
        )

        if status != int(
            CommonState.SUCCESS
        ):
            raise RuntimeError(
                f"模式切换失败：{action_name}"
            )

        print(
            f"MODE_SET_SUCCESS={action_name}"
        )


def main():

    rclpy.init()

    node = CompetitionUrsWristRoundtrip()

    current_arm = None
    current_head = None
    current_hand = [0.0, 0.0]

    switched_to_urs = False

    try:
        (
            current_arm,
            current_head,
            velocities,
        ) = node.read_state()

        subscribers = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0
            )
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command无订阅者"
            )

        publisher_count = (
            node.count_publishers(
                COMMAND_TOPIC
            )
        )

        if publisher_count > 1:
            raise RuntimeError(
                "检测到其他上肢控制发布器："
                f"{publisher_count}"
            )

        index = ARM_POS_ORDER.index(
            JOINT_NAME
        )

        current_value = float(
            current_arm[index]
        )

        target_value = (
            current_value + DELTA_RAD
        )

        lower, upper = (
            node.get_limit_map()[JOINT_NAME]
        )

        if (
            target_value
            <= lower + JOINT_LIMIT_MARGIN
        ):
            raise RuntimeError(
                "目标接近关节下限"
            )

        if (
            target_value
            >= upper - JOINT_LIMIT_MARGIN
        ):
            raise RuntimeError(
                "目标接近关节上限"
            )

        target_arm = list(current_arm)

        target_arm[index] = target_value

        print(
            "=== 比赛版X2 URS右腕10度往返测试 ==="
        )
        print(
            f"COMMAND_SUBSCRIBERS={subscribers}"
        )
        print(
            f"COMMAND_PUBLISHERS={publisher_count}"
        )
        print(
            "INPUT_SOURCE_SERVICE_USED=false"
        )
        print(
            "HAND_SUB_MODE="
            f"{UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE}"
        )
        print(
            "HAND_MODE=CLAW_OPEN_CLOSE"
        )
        print(
            "HAND_POS=0.000000,0.000000"
        )
        print(
            f"JOINT={JOINT_NAME}"
        )
        print(
            f"CURRENT={current_value:.9f}"
        )
        print(
            f"TARGET={target_value:.9f}"
        )
        print(
            f"DELTA_RAD={DELTA_RAD:+.9f}"
        )
        print(
            "DELTA_DEG=+10.000000"
        )
        print(
            f"JOINT_LOWER={lower:.9f}"
        )
        print(
            f"JOINT_UPPER={upper:.9f}"
        )
        print(
            "OTHER_ARM_JOINTS=UNCHANGED"
        )
        print(
            "HEAD_TARGET=REAL_CURRENT"
        )
        print(
            "RETURN_TO_START=true"
        )

        # 1. 原模式下先持续发送当前真实姿态。
        frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            current_hand,
            PRE_URS_HOLD_SECONDS,
        )

        print(
            f"PRE_URS_FRAMES={frames}"
        )

        # 2. 切入比赛版URS。
        node.hal_wrist_values.clear()

        node.set_mode_while_holding(
            "UPPERBODY_REMOTE_SPLIT",
            current_arm,
            current_head,
            current_hand,
        )

        switched_to_urs = True

        # 3. URS接管后保持当前姿态。
        frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            current_hand,
            POST_URS_HOLD_SECONDS,
        )

        print(
            f"POST_URS_HOLD_FRAMES={frames}"
        )

        if not node.hal_wrist_values:
            raise RuntimeError(
                "URS后未收到HAL腕部目标"
            )

        takeover_hal = float(
            node.hal_wrist_values[-1]
        )

        takeover_error = (
            takeover_hal - current_value
        )

        print(
            "TAKEOVER_HAL_WRIST="
            f"{takeover_hal:.9f}"
        )
        print(
            "TAKEOVER_ERROR="
            f"{takeover_error:+.9f}"
        )

        # 在真正运动前必须再次验证接管正确。
        if abs(takeover_error) > TAKEOVER_MAX_ERROR:
            raise RuntimeError(
                "URS接管后的HAL目标"
                "与真实腕部当前位置偏差过大"
            )

        print(
            "TAKEOVER_VALIDATED=true"
        )

        # 4. 开始10度渐变。
        node.hal_wrist_values.clear()

        frames = node.publish_segment(
            current_arm,
            target_arm,
            current_head,
            current_hand,
            RAMP_SECONDS,
        )

        print(
            f"OUTBOUND_FRAMES={frames}"
        )

        frames = node.publish_segment(
            target_arm,
            target_arm,
            current_head,
            current_hand,
            PEAK_HOLD_SECONDS,
        )

        print(
            f"PEAK_HOLD_FRAMES={frames}"
        )

        peak_arm, peak_head, _ = (
            node.read_state()
        )

        peak_position = float(
            peak_arm[index]
        )

        peak_delta = (
            peak_position - current_value
        )

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

        if node.hal_wrist_values:
            print(
                "OUTBOUND_HAL_MIN="
                f"{min(node.hal_wrist_values):.9f}"
            )
            print(
                "OUTBOUND_HAL_MAX="
                f"{max(node.hal_wrist_values):.9f}"
            )

        # 5. 无论峰值结果如何都缓慢返回。
        node.hal_wrist_values.clear()

        frames = node.publish_segment(
            target_arm,
            current_arm,
            current_head,
            current_hand,
            RAMP_SECONDS,
        )

        print(
            f"RETURN_FRAMES={frames}"
        )

        frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            current_hand,
            RETURN_HOLD_SECONDS,
        )

        print(
            f"RETURN_HOLD_FRAMES={frames}"
        )

        return_arm, _, _ = (
            node.read_state()
        )

        return_position = float(
            return_arm[index]
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

        # 6. 回到稳定站立。
        node.set_mode_while_holding(
            "STAND_DEFAULT",
            current_arm,
            current_head,
            current_hand,
        )

        switched_to_urs = False

        # ---------- 验收 ----------
        if (
            peak_delta * DELTA_RAD
            <= 0.0
        ):
            raise RuntimeError(
                "右腕实际运动方向错误"
            )

        if (
            abs(peak_delta)
            < math.radians(5.0)
        ):
            raise RuntimeError(
                "右腕实际运动幅度不足5度"
            )

        if (
            abs(peak_delta)
            > math.radians(13.0)
        ):
            raise RuntimeError(
                "右腕实际运动超过13度"
            )

        if (
            abs(return_delta)
            > math.radians(3.0)
        ):
            raise RuntimeError(
                "右腕没有返回原始位置附近"
            )

        print(
            "FINAL_MODE=STAND_DEFAULT"
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
        if (
            switched_to_urs
            and current_arm is not None
            and current_head is not None
        ):
            print(
                "RECOVERY=回到起始姿态并切STAND_DEFAULT"
            )

            try:
                node.publish_segment(
                    current_arm,
                    current_arm,
                    current_head,
                    current_hand,
                    1.0,
                )

                node.set_mode_while_holding(
                    "STAND_DEFAULT",
                    current_arm,
                    current_head,
                    current_hand,
                )

                print(
                    "RECOVERY_RESULT=PASS"
                )

            except Exception as recovery_exc:
                print(
                    "RECOVERY_RESULT=FAIL"
                )
                print(
                    "RECOVERY_REASON="
                    f"{recovery_exc}"
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
