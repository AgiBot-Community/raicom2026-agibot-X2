#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import time

import numpy as np
import rclpy

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from aimdk_msgs.msg import (
    HandStateArray,
    JointCommandArray,
)

from x2_ik_sdk import (
    ArmSide,
    X2ArmIKSolver,
    X2IKConfig,
)

from x2_safe_joint_microtest import (
    ARM_POS_ORDER,
    COMMAND_TOPIC,
)

from x2_competition_urs_wrist_roundtrip import (
    CompetitionUrsWristRoundtrip,
)


SIDE = ArmSide.RIGHT

# 右手末端：沿当前机器人基坐标系 Y- 移动 8 cm
TARGET_OFFSET = np.array(
    [0.0, -0.08, 0.0],
    dtype=float,
)

HAL_ARM_TOPIC = "/aima/hal/joint/arm/command"
HAND_STATE_TOPIC = "/aima/hal/joint/hand/state"

# 轨迹参数
PRE_URS_HOLD_SECONDS = 2.0
POST_URS_HOLD_SECONDS = 2.0

OUTBOUND_SECONDS = 6.0
TARGET_HOLD_SECONDS = 2.0

RETURN_SECONDS = 6.0
RETURN_HOLD_SECONDS = 1.5

# 真机前安全门槛
MAX_CURRENT_ARM_VELOCITY = 0.10

MAX_IK_ERROR_M = 0.002
MAX_JOINT_DELTA_DEG = 18.0
MAX_RPY_DELTA_DEG = 20.0

MIN_LIMIT_MARGIN_DEG = 5.0

# URS刚接管时：
# HAL目标必须基本等于当前真实姿态
MAX_TAKEOVER_HAL_ERROR_RAD = 0.03

# 真机结果验收
MIN_REAL_DISPLACEMENT_M = 0.05
MAX_TARGET_XYZ_ERROR_M = 0.025
MAX_RETURN_XYZ_ERROR_M = 0.020


def wrap_angle(rad):
    return math.atan2(
        math.sin(rad),
        math.cos(rad),
    )


def fmt_xyz(v):
    return (
        "["
        + ", ".join(
            f"{float(x):+.6f}"
            for x in v
        )
        + "]"
    )


class CompetitionXYZMotion(
    CompetitionUrsWristRoundtrip
):

    def __init__(self):
        super().__init__()

        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_hal_arm = {}
        self.latest_hand_state = None

        self.create_subscription(
            JointCommandArray,
            HAL_ARM_TOPIC,
            self.on_hal_full,
            qos,
        )

        self.create_subscription(
            HandStateArray,
            HAND_STATE_TOPIC,
            self.on_hand_state,
            qos,
        )

    def on_hal_full(self, msg):
        self.latest_hal_arm = {
            joint.name: float(joint.position)
            for joint in msg.joints
        }

    def on_hand_state(self, msg):
        self.latest_hand_state = msg

    def wait_for_hand_state(
        self,
        timeout_sec=5.0,
    ):
        deadline = time.monotonic() + timeout_sec

        while (
            rclpy.ok()
            and self.latest_hand_state is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

        if self.latest_hand_state is None:
            raise RuntimeError(
                "没有收到双手状态"
            )

        msg = self.latest_hand_state

        # 比赛机器人已确认左右都是 CLAW=2
        if int(msg.left_hand_type.value) != 2:
            raise RuntimeError(
                "左手不是CLAW"
            )

        if int(msg.right_hand_type.value) != 2:
            raise RuntimeError(
                "右手不是CLAW"
            )

        if not msg.left_hands:
            raise RuntimeError(
                "左夹爪状态为空"
            )

        if not msg.right_hands:
            raise RuntimeError(
                "右夹爪状态为空"
            )

        left = msg.left_hands[0]
        right = msg.right_hands[0]

        if int(left.faultcode) != 0:
            raise RuntimeError(
                f"左夹爪faultcode={left.faultcode}"
            )

        if int(right.faultcode) != 0:
            raise RuntimeError(
                f"右夹爪faultcode={right.faultcode}"
            )

        return [
            float(left.position),
            float(right.position),
        ]

    def validate_hal_takeover(
        self,
        current_arm,
    ):
        deadline = time.monotonic() + 2.0

        while (
            rclpy.ok()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

            if all(
                name in self.latest_hal_arm
                for name in ARM_POS_ORDER
            ):
                break

        if not all(
            name in self.latest_hal_arm
            for name in ARM_POS_ORDER
        ):
            raise RuntimeError(
                "URS后没有获得完整14维HAL目标"
            )

        errors = []

        for name, target in zip(
            ARM_POS_ORDER,
            current_arm,
        ):
            hal = self.latest_hal_arm[name]

            errors.append(
                abs(
                    hal - float(target)
                )
            )

        max_error = max(errors)

        max_index = int(
            np.argmax(errors)
        )

        print(
            "TAKEOVER_HAL_MAX_ERROR_RAD="
            f"{max_error:.9f}"
        )

        print(
            "TAKEOVER_HAL_MAX_ERROR_JOINT="
            f"{ARM_POS_ORDER[max_index]}"
        )

        if (
            max_error
            > MAX_TAKEOVER_HAL_ERROR_RAD
        ):
            raise RuntimeError(
                "URS接管后HAL目标"
                "与当前真实姿态偏差过大"
            )

        print(
            "TAKEOVER_VALIDATED=true"
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--execute",
        action="store_true",
        help="允许执行真实XYZ运动",
    )

    args = parser.parse_args()

    rclpy.init()

    node = CompetitionXYZMotion()

    solver = X2ArmIKSolver(
        X2IKConfig.default_omnipicker()
    )

    current_arm = None
    current_head = None
    current_hand = None

    switched_to_urs = False

    try:
        # ========================================
        # 1. 读取真实状态
        # ========================================

        (
            current_arm,
            current_head,
            velocities,
        ) = node.read_state()

        current_arm = np.asarray(
            current_arm,
            dtype=float,
        )

        current_head = np.asarray(
            current_head,
            dtype=float,
        )

        if current_arm.shape != (14,):
            raise RuntimeError(
                "真实arm_pos不是14维"
            )

        # read_state() 在当前比赛机版本中可能返回
        # dict，而不是简单的14维velocity数组。
        print(
            "VELOCITY_CONTAINER_TYPE="
            f"{type(velocities).__name__}"
        )

        velocity_values = []

        if isinstance(velocities, dict):
            for joint_name, value in velocities.items():

                if isinstance(value, dict):
                    if "velocity" not in value:
                        continue

                    velocity_values.append(
                        float(value["velocity"])
                    )

                elif hasattr(value, "velocity"):
                    velocity_values.append(
                        float(value.velocity)
                    )

                else:
                    velocity_values.append(
                        float(value)
                    )

        else:
            velocity_array = np.asarray(
                velocities,
                dtype=float,
            ).reshape(-1)

            velocity_values = [
                float(x)
                for x in velocity_array
            ]

        if not velocity_values:
            raise RuntimeError(
                "没有解析到有效的机械臂速度"
            )

        max_velocity = max(
            abs(v)
            for v in velocity_values
        )

        print(
            "MAX_CURRENT_ARM_VELOCITY="
            f"{max_velocity:.9f}"
        )

        if (
            max_velocity
            > MAX_CURRENT_ARM_VELOCITY
        ):
            raise RuntimeError(
                "机械臂当前仍在运动，"
                f"max_velocity={max_velocity:.6f}"
            )

        current_hand = (
            node.wait_for_hand_state()
        )

        # ========================================
        # 2. FK：计算实时起点
        # ========================================

        start_xyz = np.asarray(
            solver.fk_xyz(
                SIDE,
                current_arm,
                current_head_pos=current_head,
            ),
            dtype=float,
        )

        start_rpy = np.asarray(
            solver.fk_rpy(
                SIDE,
                current_arm,
                current_head_pos=current_head,
            ),
            dtype=float,
        )

        target_xyz = (
            start_xyz
            + TARGET_OFFSET
        )

        # ========================================
        # 3. IK
        # ========================================

        result = solver.solve_position(
            SIDE,
            target_xyz,
            current_arm,
            current_head_pos=current_head,
        )

        if not result.success:
            raise RuntimeError(
                "IK未收敛: "
                f"{result.message}"
            )

        target_arm = np.asarray(
            result.arm_pos,
            dtype=float,
        )

        final_xyz = np.asarray(
            result.final_xyz,
            dtype=float,
        )

        ik_error = float(
            np.linalg.norm(
                final_xyz - target_xyz
            )
        )

        # ========================================
        # 4. 关节变化检查
        # ========================================

        delta = (
            target_arm
            - current_arm
        )

        delta_deg = np.degrees(
            delta
        )

        right_delta_deg = (
            delta_deg[7:]
        )

        left_delta_deg = (
            delta_deg[:7]
        )

        max_right_idx = int(
            np.argmax(
                np.abs(
                    right_delta_deg
                )
            )
        )

        max_joint_delta = float(
            abs(
                right_delta_deg[
                    max_right_idx
                ]
            )
        )

        max_joint_name = (
            ARM_POS_ORDER[
                7 + max_right_idx
            ]
        )

        left_max_delta = float(
            np.max(
                np.abs(
                    left_delta_deg
                )
            )
        )

        if left_max_delta > 0.5:
            raise RuntimeError(
                "IK意外改变左臂"
            )

        if (
            max_joint_delta
            > MAX_JOINT_DELTA_DEG
        ):
            raise RuntimeError(
                "IK最大关节变化过大："
                f"{max_joint_delta:.3f}deg"
            )

        # ========================================
        # 5. 目标关节限位检查
        # ========================================

        limits = (
            solver.joint_limits_for_arm_pos()
        )

        right_limits = limits[7:]

        margins = []

        for q, (
            name,
            lower,
            upper,
        ) in zip(
            target_arm[7:],
            right_limits,
        ):
            margin = min(
                float(q) - float(lower),
                float(upper) - float(q),
            )

            margins.append(
                (
                    name,
                    float(margin),
                )
            )

        min_margin_name, min_margin = min(
            margins,
            key=lambda x: x[1],
        )

        min_margin_deg = math.degrees(
            min_margin
        )

        if (
            min_margin_deg
            < MIN_LIMIT_MARGIN_DEG
        ):
            raise RuntimeError(
                "目标太接近关节限位："
                f"{min_margin_name}, "
                f"margin={min_margin_deg:.3f}deg"
            )

        # ========================================
        # 6. RPY变化检查
        # ========================================

        target_rpy = np.asarray(
            solver.fk_rpy(
                SIDE,
                target_arm,
                current_head_pos=current_head,
            ),
            dtype=float,
        )

        rpy_delta = np.asarray(
            [
                wrap_angle(
                    target_rpy[i]
                    - start_rpy[i]
                )
                for i in range(3)
            ]
        )

        rpy_delta_deg = np.degrees(
            rpy_delta
        )

        max_rpy_delta = float(
            np.max(
                np.abs(
                    rpy_delta_deg
                )
            )
        )

        if (
            max_rpy_delta
            > MAX_RPY_DELTA_DEG
        ):
            raise RuntimeError(
                "position-only IK导致"
                "末端姿态变化过大："
                f"{max_rpy_delta:.3f}deg"
            )

        if ik_error > MAX_IK_ERROR_M:
            raise RuntimeError(
                "IK位置误差过大："
                f"{ik_error:.6f}m"
            )

        # ========================================
        # 7. 输出dry-run信息
        # ========================================

        print(
            "=== 比赛版X2右臂XYZ Y- 8cm测试 ==="
        )

        print(
            f"EXECUTE={str(args.execute).lower()}"
        )

        print(
            "INPUT_SOURCE_SERVICE_USED=false"
        )

        print(
            "SIDE=RIGHT"
        )

        print(
            f"START_XYZ={fmt_xyz(start_xyz)}"
        )

        print(
            "TARGET_OFFSET_M="
            f"{fmt_xyz(TARGET_OFFSET)}"
        )

        print(
            f"TARGET_XYZ={fmt_xyz(target_xyz)}"
        )

        print(
            f"IK_SUCCESS={result.success}"
        )

        print(
            f"IK_ITERATIONS={result.iterations}"
        )

        print(
            f"IK_FINAL_XYZ={fmt_xyz(final_xyz)}"
        )

        print(
            f"IK_ERROR_M={ik_error:.9f}"
        )

        print(
            "RIGHT_JOINT_DELTA_DEG=["
            + ", ".join(
                f"{x:+.3f}"
                for x in right_delta_deg
            )
            + "]"
        )

        print(
            f"MAX_JOINT_DELTA_DEG={max_joint_delta:.3f}"
        )

        print(
            f"MAX_JOINT={max_joint_name}"
        )

        print(
            f"LEFT_ARM_MAX_DELTA_DEG={left_max_delta:.6f}"
        )

        print(
            "RPY_DELTA_DEG=["
            + ", ".join(
                f"{x:+.3f}"
                for x in rpy_delta_deg
            )
            + "]"
        )

        print(
            f"MAX_RPY_DELTA_DEG={max_rpy_delta:.3f}"
        )

        print(
            "MIN_JOINT_LIMIT_MARGIN_DEG="
            f"{min_margin_deg:.3f}"
        )

        print(
            "MIN_JOINT_LIMIT_MARGIN_JOINT="
            f"{min_margin_name}"
        )

        print(
            "HAND_POS="
            + ",".join(
                f"{x:.6f}"
                for x in current_hand
            )
        )

        print(
            "OTHER_ARM=UNCHANGED"
        )

        print(
            "TRAJECTORY_TYPE=JOINT_SPACE_LINEAR"
        )

        print(
            f"OUTBOUND_SECONDS={OUTBOUND_SECONDS}"
        )

        print(
            f"RETURN_SECONDS={RETURN_SECONDS}"
        )

        if not args.execute:
            print(
                "REAL_ROBOT_MOTION=false"
            )
            print(
                "RESULT=PASS"
            )
            return 0

        # ========================================
        # 真机部分
        # ========================================

        subscribers = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0
            )
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command无订阅者"
            )

        publishers = node.count_publishers(
            COMMAND_TOPIC
        )

        # 当前程序自己的publisher会算一个。
        if publishers > 1:
            raise RuntimeError(
                "存在其他上肢命令发布器："
                f"{publishers}"
            )

        print(
            f"COMMAND_SUBSCRIBERS={subscribers}"
        )

        print(
            f"COMMAND_PUBLISHERS={publishers}"
        )

        print(
            "REAL_ROBOT_MOTION=true"
        )

        # ========================================
        # 8. URS前保持当前真实姿态
        # ========================================

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

        node.latest_hal_arm.clear()

        # ========================================
        # 9. 切入比赛版上肢分离模式
        # ========================================

        node.set_mode_while_holding(
            "UPPERBODY_REMOTE_SPLIT",
            current_arm,
            current_head,
            current_hand,
        )

        switched_to_urs = True

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

        # 真正运动前再次验证HAL
        node.validate_hal_takeover(
            current_arm
        )

        # ========================================
        # 10. 6秒移动到IK目标
        # ========================================

        print(
            "XYZ_MOTION_START=true"
        )

        frames = node.publish_segment(
            current_arm,
            target_arm,
            current_head,
            current_hand,
            OUTBOUND_SECONDS,
        )

        print(
            f"OUTBOUND_FRAMES={frames}"
        )

        frames = node.publish_segment(
            target_arm,
            target_arm,
            current_head,
            current_hand,
            TARGET_HOLD_SECONDS,
        )

        print(
            f"TARGET_HOLD_FRAMES={frames}"
        )

        # ========================================
        # 11. 实际终点验证
        # ========================================

        (
            peak_arm,
            peak_head,
            peak_velocities,
        ) = node.read_state()

        peak_arm = np.asarray(
            peak_arm,
            dtype=float,
        )

        peak_head = np.asarray(
            peak_head,
            dtype=float,
        )

        real_peak_xyz = np.asarray(
            solver.fk_xyz(
                SIDE,
                peak_arm,
                current_head_pos=peak_head,
            ),
            dtype=float,
        )

        real_displacement = (
            real_peak_xyz
            - start_xyz
        )

        real_distance = float(
            np.linalg.norm(
                real_displacement
            )
        )

        target_error = float(
            np.linalg.norm(
                real_peak_xyz
                - target_xyz
            )
        )

        real_target_joint_error_deg = float(
            np.max(
                np.abs(
                    np.degrees(
                        peak_arm[7:]
                        - target_arm[7:]
                    )
                )
            )
        )

        print(
            f"REAL_PEAK_XYZ={fmt_xyz(real_peak_xyz)}"
        )

        print(
            "REAL_XYZ_DISPLACEMENT="
            f"{fmt_xyz(real_displacement)}"
        )

        print(
            "REAL_XYZ_DISPLACEMENT_M="
            f"{real_distance:.6f}"
        )

        print(
            "REAL_XYZ_DISPLACEMENT_CM="
            f"{real_distance * 100.0:.3f}"
        )

        print(
            "REAL_TARGET_XYZ_ERROR_M="
            f"{target_error:.6f}"
        )

        print(
            "REAL_TARGET_JOINT_MAX_ERROR_DEG="
            f"{real_target_joint_error_deg:.3f}"
        )

        # ========================================
        # 12. 无论最终验收如何，先正常返回
        # ========================================

        print(
            "RETURN_START=true"
        )

        frames = node.publish_segment(
            target_arm,
            current_arm,
            current_head,
            current_hand,
            RETURN_SECONDS,
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

        (
            return_arm,
            return_head,
            _,
        ) = node.read_state()

        return_arm = np.asarray(
            return_arm,
            dtype=float,
        )

        return_head = np.asarray(
            return_head,
            dtype=float,
        )

        return_xyz = np.asarray(
            solver.fk_xyz(
                SIDE,
                return_arm,
                current_head_pos=return_head,
            ),
            dtype=float,
        )

        return_error = float(
            np.linalg.norm(
                return_xyz - start_xyz
            )
        )

        print(
            f"RETURN_REAL_XYZ={fmt_xyz(return_xyz)}"
        )

        print(
            "RETURN_XYZ_ERROR_M="
            f"{return_error:.6f}"
        )

        print(
            "RETURN_XYZ_ERROR_CM="
            f"{return_error * 100.0:.3f}"
        )

        # ========================================
        # 13. 回稳定站立
        # ========================================

        node.set_mode_while_holding(
            "STAND_DEFAULT",
            current_arm,
            current_head,
            current_hand,
        )

        switched_to_urs = False

        print(
            "FINAL_MODE=STAND_DEFAULT"
        )

        # ========================================
        # 14. 真机结果验收
        # ========================================

        if (
            real_distance
            < MIN_REAL_DISPLACEMENT_M
        ):
            raise RuntimeError(
                "真实XYZ移动不足5cm"
            )

        # 目标方向必须明显是Y-
        if (
            real_displacement[1]
            > -0.04
        ):
            raise RuntimeError(
                "真实末端没有明显沿Y-方向运动"
            )

        if (
            target_error
            > MAX_TARGET_XYZ_ERROR_M
        ):
            raise RuntimeError(
                "真实末端目标误差过大："
                f"{target_error:.6f}m"
            )

        if (
            return_error
            > MAX_RETURN_XYZ_ERROR_M
        ):
            raise RuntimeError(
                "返回起点误差过大："
                f"{return_error:.6f}m"
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

        # 如果中途异常且已经进入URS：
        # 从当前真实状态缓慢回起点，不做位置突跳。
        if (
            switched_to_urs
            and current_arm is not None
            and current_head is not None
            and current_hand is not None
        ):
            print(
                "RECOVERY=尝试缓慢返回起点并切STAND_DEFAULT"
            )

            try:
                (
                    actual_arm,
                    actual_head,
                    _,
                ) = node.read_state()

                actual_arm = np.asarray(
                    actual_arm,
                    dtype=float,
                )

                node.publish_segment(
                    actual_arm,
                    current_arm,
                    current_head,
                    current_hand,
                    4.0,
                )

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
