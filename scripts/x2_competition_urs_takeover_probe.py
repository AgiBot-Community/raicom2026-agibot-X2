#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
HAND_STATE_TOPIC = "/aima/hal/joint/hand/state"

PUBLISH_RATE_HZ = 50.0
PRE_HOLD_SECONDS = 2.0
URS_HOLD_SECONDS = 3.0

WATCH_JOINT = "right_wrist_roll_joint"


class CompetitionUrsProbe(SafeJointMicrotest):

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

        self.hal_values = []

        self.create_subscription(
            JointCommandArray,
            HAL_ARM_TOPIC,
            self.on_hal,
            qos,
        )

        self.sequence = 0

    def on_hal(self, msg):
        for joint in msg.joints:
            if joint.name == WATCH_JOINT:
                self.hal_values.append(
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

        # 完全跟本机官方upper_body_control保持一致
        msg.source = "upper_body_example"

        # 比赛机器人当前是双CLAW
        msg.hand_sub_mode = (
            UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE
        )

        msg.head_pos = [
            float(x)
            for x in head_pos
        ]

        msg.arm_pos = [
            float(x)
            for x in arm_pos
        ]

        msg.hand_pos = [
            float(x)
            for x in hand_pos
        ]

        self.command_pub.publish(msg)

    def hold(
        self,
        arm_pos,
        head_pos,
        hand_pos,
        seconds,
    ):
        period = 1.0 / PUBLISH_RATE_HZ
        deadline = time.monotonic() + seconds
        frames = 0

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            self.publish_upper(
                arm_pos,
                head_pos,
                hand_pos,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.001,
            )

            frames += 1
            time.sleep(period)

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

        future = self.mode_client.call_async(req)

        deadline = time.monotonic() + 5.0
        period = 1.0 / PUBLISH_RATE_HZ

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

            time.sleep(period)

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

        if status != int(CommonState.SUCCESS):
            raise RuntimeError(
                f"模式切换失败：{action_name}"
            )

        print(
            f"MODE_SET_SUCCESS={action_name}"
        )


def main():

    rclpy.init()

    node = CompetitionUrsProbe()

    current_arm = None
    current_head = None
    switched = False

    try:
        current_arm, current_head, velocities = (
            node.read_state()
        )

        subscribers = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0
            )
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command无订阅者"
            )

        # 当前HandState已经确认左右均为CLAW，
        # 且当前position都是0.0。
        current_hand = [0.0, 0.0]

        index = ARM_POS_ORDER.index(
            WATCH_JOINT
        )

        current_wrist = float(
            current_arm[index]
        )

        print(
            "=== 比赛版X2 URS接管探针 ==="
        )
        print(
            f"COMMAND_SUBSCRIBERS={subscribers}"
        )
        print(
            "HAND_SUB_MODE="
            f"{UpperBodyCommandArray.HAND_CLAW_OPEN_CLOSE}"
        )
        print(
            "HAND_MODE=CLAW_OPEN_CLOSE"
        )
        print(
            "HAND_POS="
            + ",".join(
                f"{x:.6f}"
                for x in current_hand
            )
        )
        print(
            f"WATCH_JOINT={WATCH_JOINT}"
        )
        print(
            f"REAL_JOINT_CURRENT={current_wrist:.9f}"
        )
        print(
            "HEAD_CURRENT="
            + ",".join(
                f"{x:.9f}"
                for x in current_head
            )
        )
        print(
            "ARM_TARGET=REAL_CURRENT_STATE"
        )
        print(
            "JOINT_MOTION_COMMAND=false"
        )
        print(
            "INPUT_SOURCE_SERVICE_USED=false"
        )

        frames = node.hold(
            current_arm,
            current_head,
            current_hand,
            PRE_HOLD_SECONDS,
        )

        print(
            f"PRE_URS_FRAMES={frames}"
        )

        node.hal_values.clear()

        node.set_mode_while_holding(
            "UPPERBODY_REMOTE_SPLIT",
            current_arm,
            current_head,
            current_hand,
        )

        switched = True

        frames = node.hold(
            current_arm,
            current_head,
            current_hand,
            URS_HOLD_SECONDS,
        )

        print(
            f"URS_HOLD_FRAMES={frames}"
        )

        if node.hal_values:
            print(
                "HAL_WRIST_FIRST="
                f"{node.hal_values[0]:.9f}"
            )
            print(
                "HAL_WRIST_LAST="
                f"{node.hal_values[-1]:.9f}"
            )
            print(
                "HAL_WRIST_MIN="
                f"{min(node.hal_values):.9f}"
            )
            print(
                "HAL_WRIST_MAX="
                f"{max(node.hal_values):.9f}"
            )
        else:
            print(
                "HAL_WRIST_DATA=NONE"
            )

        node.set_mode_while_holding(
            "STAND_DEFAULT",
            current_arm,
            current_head,
            current_hand,
        )

        switched = False

        print(
            "JOINT_MOTION_COMMAND=false"
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
            switched
            and current_arm is not None
            and current_head is not None
        ):
            try:
                node.set_mode_while_holding(
                    "STAND_DEFAULT",
                    current_arm,
                    current_head,
                    [0.0, 0.0],
                )

                print(
                    "RECOVERY_MODE=STAND_DEFAULT"
                )

            except Exception as exc:
                print(
                    "RECOVERY_FAILED="
                    f"{exc}"
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
