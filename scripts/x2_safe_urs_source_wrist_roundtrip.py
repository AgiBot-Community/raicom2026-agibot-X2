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
    McInputAction,
    MessageHeader,
    RequestHeader,
    UpperBodyCommandArray,
)
from aimdk_msgs.srv import (
    GetCurrentInputSource,
    SetMcAction,
    SetMcInputSource,
)

from x2_safe_joint_microtest import (
    ARM_POS_ORDER,
    COMMAND_TOPIC,
    SafeJointMicrotest,
)


MODE_SERVICE = "/aimdk_5Fmsgs/srv/SetMcAction"
INPUT_SET_SERVICE = "/aimdk_5Fmsgs/srv/SetMcInputSource"
INPUT_GET_SERVICE = "/aimdk_5Fmsgs/srv/GetCurrentInputSource"
HAL_ARM_TOPIC = "/aima/hal/joint/arm/command"

INPUT_SOURCE_NAME = "upper_body_example"
INPUT_SOURCE_PRIORITY = 40
INPUT_SOURCE_TIMEOUT_MS = 1000

JOINT_NAME = "right_wrist_roll_joint"
DELTA_RAD = math.radians(10.0)

PRE_REGISTER_HOLD_SECONDS = 2.0
POST_REGISTER_HOLD_SECONDS = 1.0
POST_MODE_HOLD_SECONDS = 2.0
RAMP_SECONDS = 5.0
PEAK_HOLD_SECONDS = 1.5
RETURN_HOLD_SECONDS = 1.5

PUBLISH_RATE_HZ = 50.0
JOINT_LIMIT_MARGIN = 0.03


class SafeUrsSourceTest(SafeJointMicrotest):
    def __init__(self) -> None:
        super().__init__()

        self.mode_client = self.create_client(
            SetMcAction,
            MODE_SERVICE,
        )

        self.input_set_client = self.create_client(
            SetMcInputSource,
            INPUT_SET_SERVICE,
        )

        self.input_get_client = self.create_client(
            GetCurrentInputSource,
            INPUT_GET_SERVICE,
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
            self.on_hal_command,
            qos,
        )

        self.sequence = 0

    def on_hal_command(
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

        # 必须与SetMcInputSource注册名称一致。
        message.source = INPUT_SOURCE_NAME

        # 本测试不控制手部。
        message.hand_sub_mode = 0
        message.hand_pos = []

        # 保持当前真实头部姿态。
        message.head_pos = [
            float(value)
            for value in head_position
        ]

        # 完整14维手臂目标。
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
            elapsed = time.monotonic() - start_time

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

            remaining = next_tick - time.monotonic()

            if remaining > 0.0:
                time.sleep(remaining)

        return frames

    def wait_future_while_holding(
        self,
        future,
        hold_arm,
        hold_head,
        timeout_sec: float,
    ) -> bool:
        period = 1.0 / PUBLISH_RATE_HZ
        deadline = time.monotonic() + timeout_sec
        next_tick = time.monotonic()

        while (
            rclpy.ok()
            and not future.done()
            and time.monotonic() < deadline
        ):
            self.publish_once(
                arm_position=hold_arm,
                head_position=hold_head,
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.005,
            )

            next_tick += period
            remaining = next_tick - time.monotonic()

            if remaining > 0.0:
                time.sleep(remaining)

        return future.done()

    def call_input_action(
        self,
        action_value: int,
        hold_arm,
        hold_head,
    ) -> bool:
        if not self.input_set_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"输入源服务不可用：{INPUT_SET_SERVICE}"
            )

        action_name = {
            McInputAction.INPUTACTION_ADD: "ADD",
            McInputAction.INPUTACTION_MODIFY: "MODIFY",
            McInputAction.INPUTACTION_ENABLE: "ENABLE",
            McInputAction.INPUTACTION_DISABLE: "DISABLE",
            McInputAction.INPUTACTION_DELETE: "DELETE",
        }.get(action_value, str(action_value))

        response = None

        for attempt in range(8):
            request = SetMcInputSource.Request()
            request.request.header.stamp = (
                self.get_clock().now().to_msg()
            )

            request.action = McInputAction()
            request.action.value = action_value

            request.input_source.name = (
                INPUT_SOURCE_NAME
            )
            request.input_source.priority = (
                INPUT_SOURCE_PRIORITY
            )
            request.input_source.timeout = (
                INPUT_SOURCE_TIMEOUT_MS
            )

            future = self.input_set_client.call_async(
                request
            )

            completed = self.wait_future_while_holding(
                future=future,
                hold_arm=hold_arm,
                hold_head=hold_head,
                timeout_sec=0.35,
            )

            if completed:
                response = future.result()
                break

            print(
                f"INPUT_ACTION_RETRY="
                f"{action_name}:{attempt + 1}"
            )

        if response is None:
            print(
                f"INPUT_ACTION_RESULT="
                f"{action_name}:TIMEOUT"
            )
            return False

        code = int(
            response.response.header.code
        )

        print(
            f"INPUT_ACTION_RESULT="
            f"{action_name}:code={code}"
        )

        return code == 0

    def register_input_source(
        self,
        hold_arm,
        hold_head,
    ) -> None:
        # 首次运行时直接ADD。
        if self.call_input_action(
            McInputAction.INPUTACTION_ADD,
            hold_arm,
            hold_head,
        ):
            print(
                "INPUT_SOURCE_REGISTERED=ADD"
            )
            return

        # 已存在时不能只ENABLE，因为ENABLE不会改变priority。
        print(
            "INPUT_SOURCE_ADD_FAILED="
            "trying_MODIFY"
        )

        if not self.call_input_action(
            McInputAction.INPUTACTION_MODIFY,
            hold_arm,
            hold_head,
        ):
            raise RuntimeError(
                "已有输入源MODIFY失败"
            )

        print(
            "INPUT_SOURCE_MODIFIED=true"
        )

        # MODIFY后显式ENABLE，确保它不是disabled状态。
        if not self.call_input_action(
            McInputAction.INPUTACTION_ENABLE,
            hold_arm,
            hold_head,
        ):
            raise RuntimeError(
                "输入源MODIFY成功，但ENABLE失败"
            )

        print(
            "INPUT_SOURCE_REGISTERED="
            "MODIFY_AND_ENABLE"
        )

    def disable_input_source(
        self,
        hold_arm,
        hold_head,
    ) -> bool:
        result = self.call_input_action(
            McInputAction.INPUTACTION_DISABLE,
            hold_arm,
            hold_head,
        )

        print(
            "INPUT_SOURCE_DISABLED="
            f"{str(result).lower()}"
        )

        return result

    def get_current_input_source(
        self,
        hold_arm,
        hold_head,
    ):
        if not self.input_get_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"输入源查询服务不可用："
                f"{INPUT_GET_SERVICE}"
            )

        request = GetCurrentInputSource.Request()
        request.request.header.stamp = (
            self.get_clock().now().to_msg()
        )

        future = self.input_get_client.call_async(
            request
        )

        completed = self.wait_future_while_holding(
            future=future,
            hold_arm=hold_arm,
            hold_head=hold_head,
            timeout_sec=3.0,
        )

        if not completed:
            raise RuntimeError(
                "查询当前输入源超时"
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                "查询当前输入源返回空响应"
            )

        code = int(
            response.response.header.code
        )

        if code != 0:
            raise RuntimeError(
                f"查询输入源失败：code={code}"
            )

        source = response.input_source

        print(f"CURRENT_INPUT_NAME={source.name}")
        print(
            "CURRENT_INPUT_PRIORITY="
            f"{source.priority}"
        )
        print(
            "CURRENT_INPUT_TIMEOUT="
            f"{source.timeout}"
        )

        return (
            str(source.name),
            int(source.priority),
            int(source.timeout),
        )

    def set_mode_while_holding(
        self,
        action_name: str,
        hold_arm,
        hold_head,
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

        future = self.mode_client.call_async(
            request
        )

        completed = self.wait_future_while_holding(
            future=future,
            hold_arm=hold_arm,
            hold_head=hold_head,
            timeout_sec=5.0,
        )

        if not completed:
            raise RuntimeError(
                f"切换模式超时：{action_name}"
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                f"切换模式返回空响应："
                f"{action_name}"
            )

        status = int(
            response.response.status.value
        )
        message = str(
            response.response.message
        )

        print(f"MODE_RESPONSE_STATUS={status}")
        print(
            "MODE_RESPONSE_MESSAGE="
            f"{message!r}"
        )

        if status != int(CommonState.SUCCESS):
            raise RuntimeError(
                f"模式切换失败：{action_name}，"
                f"status={status}，"
                f"message={message!r}"
            )

        print(
            f"MODE_SET_SUCCESS={action_name}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "注册MC输入源并执行URS右腕10度安全往返"
        )
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="显式允许注册输入源、切换URS并执行动作",
    )

    parser.add_argument(
        "--takeover-only",
        action="store_true",
        help="只验证输入源和URS接管，不执行10度关节动作",
    )

    args = parser.parse_args()

    rclpy.init()
    node = SafeUrsSourceTest()

    current_arm = None
    current_head = None
    source_registered = False
    switched_to_urs = False

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
                f"{COMMAND_TOPIC}没有订阅者"
            )

        publisher_count = node.count_publishers(
            COMMAND_TOPIC
        )

        # 本节点自身通常计入一个publisher。
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
        target_value = (
            current_value + DELTA_RAD
        )

        lower, upper = node.get_limit_map()[
            JOINT_NAME
        ]

        if target_value <= lower + JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                "目标过于接近关节下限"
            )

        if target_value >= upper - JOINT_LIMIT_MARGIN:
            raise RuntimeError(
                "目标过于接近关节上限"
            )

        target_arm = list(current_arm)
        target_arm[joint_index] = target_value

        print(
            "=== X2 URS输入源注册及右腕往返测试 ==="
        )
        print(f"COMMAND_SUBSCRIBERS={subscribers}")
        print(f"COMMAND_PUBLISHERS={publisher_count}")
        print(f"INPUT_SOURCE_NAME={INPUT_SOURCE_NAME}")
        print(
            "INPUT_SOURCE_PRIORITY="
            f"{INPUT_SOURCE_PRIORITY}"
        )
        print(
            "INPUT_SOURCE_TIMEOUT_MS="
            f"{INPUT_SOURCE_TIMEOUT_MS}"
        )
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
        print("RETURN_TO_START=true")
        print("FINAL_MODE=STAND_DEFAULT")

        if not args.execute:
            print("DRY_RUN=true")
            print("INPUT_SOURCE_CHANGED=false")
            print("MODE_CHANGED=false")
            print("REAL_COMMAND_PUBLISHED=false")
            print("RESULT=PASS")
            return 0

        print("DRY_RUN=false")
        print("REAL_TEST_STARTED=true")

        # 1. 先持续发布当前姿态。
        frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            PRE_REGISTER_HOLD_SECONDS,
        )
        print(
            "PRE_REGISTER_HOLD_FRAMES="
            f"{frames}"
        )

        # 2. 注册或启用消息对应的输入源。
        node.register_input_source(
            current_arm,
            current_head,
        )
        source_registered = True

        frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            POST_REGISTER_HOLD_SECONDS,
        )
        print(
            "POST_REGISTER_HOLD_FRAMES="
            f"{frames}"
        )

        # 3. URS切换前只记录当前输入源。
        # 此时upper-body控制尚未进入URS，不把当前源仍为rc视为失败。
        source_name, _, _ = (
            node.get_current_input_source(
                current_arm,
                current_head,
            )
        )

        print(f"PRE_URS_INPUT_NAME={source_name}")

        # 4. 保持当前姿态时切换URS。
        node.set_mode_while_holding(
            "UPPERBODY_REMOTE_SPLIT",
            current_arm,
            current_head,
        )
        switched_to_urs = True

        frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            POST_MODE_HOLD_SECONDS,
        )
        print(
            "POST_MODE_HOLD_FRAMES="
            f"{frames}"
        )

        # 切换URS后再次确认控制源。
        source_name, _, _ = (
            node.get_current_input_source(
                current_arm,
                current_head,
            )
        )

        if source_name != INPUT_SOURCE_NAME:
            raise RuntimeError(
                "进入URS并持续发布上肢命令后，"
                "当前输入源仍不是测试源："
                f"{source_name!r}"
            )

        print("POST_URS_INPUT_SOURCE_CONTROL_CONFIRMED=true")

        # 只验证接管，不执行任何关节位移。
        if args.takeover_only:
            print("TAKEOVER_ONLY=true")
            print("JOINT_MOTION_COMMAND_SENT=false")

            node.set_mode_while_holding(
                "STAND_DEFAULT",
                current_arm,
                current_head,
            )
            switched_to_urs = False

            node.publish_segment(
                current_arm,
                current_arm,
                current_head,
                1.0,
            )

            node.disable_input_source(
                current_arm,
                current_head,
            )
            source_registered = False

            print("RESULT=PASS")
            return 0

        node.hal_wrist_values.clear()

        # 5. 右腕缓慢运动10度。
        outbound_frames = node.publish_segment(
            current_arm,
            target_arm,
            current_head,
            RAMP_SECONDS,
        )
        print(
            f"OUTBOUND_FRAMES={outbound_frames}"
        )

        peak_hold_frames = node.publish_segment(
            target_arm,
            target_arm,
            current_head,
            PEAK_HOLD_SECONDS,
        )
        print(
            "PEAK_HOLD_FRAMES="
            f"{peak_hold_frames}"
        )

        peak_arm, peak_head, _ = (
            node.read_state()
        )

        peak_position = float(
            peak_arm[joint_index]
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
        print(
            "PEAK_OBSERVED_HEAD="
            + ",".join(
                f"{value:.9f}"
                for value in peak_head
            )
        )

        # 6. 缓慢返回原始位置。
        return_frames = node.publish_segment(
            target_arm,
            current_arm,
            current_head,
            RAMP_SECONDS,
        )
        print(
            f"RETURN_FRAMES={return_frames}"
        )

        return_hold_frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            RETURN_HOLD_SECONDS,
        )
        print(
            "RETURN_HOLD_FRAMES="
            f"{return_hold_frames}"
        )

        return_arm, _, _ = node.read_state()

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

        # 7. 保持原姿态并切回稳定站立。
        node.set_mode_while_holding(
            "STAND_DEFAULT",
            current_arm,
            current_head,
        )
        switched_to_urs = False

        frames = node.publish_segment(
            current_arm,
            current_arm,
            current_head,
            1.0,
        )
        print(f"FINAL_HOLD_FRAMES={frames}")

        # 8. 退出前禁用本测试输入源。
        node.disable_input_source(
            current_arm,
            current_head,
        )
        source_registered = False

        if node.hal_wrist_values:
            hal_min = min(node.hal_wrist_values)
            hal_max = max(node.hal_wrist_values)

            print(f"HAL_WRIST_MIN={hal_min:.9f}")
            print(f"HAL_WRIST_MAX={hal_max:.9f}")
            print(
                "HAL_WRIST_RANGE="
                f"{hal_max - hal_min:.9f}"
            )
        else:
            print("HAL_WRIST_DATA=NONE")

        if abs(peak_delta) < math.radians(2.0):
            print("RESULT=FAIL")
            print(
                "REASON=取得输入源和URS后，"
                "实际运动仍小于2度"
            )
            return 1

        if peak_delta * DELTA_RAD <= 0.0:
            print("RESULT=FAIL")
            print(
                "REASON=实际运动方向错误"
            )
            return 1

        if abs(peak_delta) > math.radians(13.0):
            print("RESULT=FAIL")
            print(
                "REASON=实际运动明显超过10度"
            )
            return 1

        if abs(return_delta) > math.radians(3.0):
            print("RESULT=FAIL")
            print(
                "REASON=右腕未返回原位附近"
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
                "RECOVERY=尝试切回STAND_DEFAULT"
            )

            try:
                node.publish_segment(
                    current_arm,
                    current_arm,
                    current_head,
                    1.0,
                )

                node.set_mode_while_holding(
                    "STAND_DEFAULT",
                    current_arm,
                    current_head,
                )

                print("RECOVERY_MODE_RESULT=PASS")

            except Exception as recovery_exc:
                print("RECOVERY_MODE_RESULT=FAIL")
                print(
                    f"RECOVERY_MODE_REASON="
                    f"{recovery_exc}"
                )

        if (
            source_registered
            and current_arm is not None
            and current_head is not None
        ):
            print(
                "RECOVERY=尝试禁用测试输入源"
            )

            try:
                ok = node.disable_input_source(
                    current_arm,
                    current_head,
                )

                print(
                    "RECOVERY_INPUT_RESULT="
                    f"{str(ok).upper()}"
                )

            except Exception as recovery_exc:
                print("RECOVERY_INPUT_RESULT=FAIL")
                print(
                    f"RECOVERY_INPUT_REASON="
                    f"{recovery_exc}"
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
