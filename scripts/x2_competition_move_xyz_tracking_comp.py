#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys

import numpy as np
import rclpy

from x2_ik_sdk import (
    ArmSide,
    X2ArmIKSolver,
    X2IKConfig,
)

from x2_safe_joint_microtest import (
    ARM_POS_ORDER,
    COMMAND_TOPIC,
)

from x2_competition_move_xyz_closed_loop import (
    XYZClosedLoopNode,
    measure_robot,
    plan_cartesian_path,
    print_plan,
    execute_plan,
    extract_max_velocity,
    fmt_xyz,
    wrap_angle,
    WAYPOINT_STEP_M,
    CARTESIAN_SPEED_MPS,
    PRE_URS_HOLD_SECONDS,
    POST_URS_HOLD_SECONDS,
)


SIDE = ArmSide.RIGHT

# -------------------------------
# 基础安全参数
# -------------------------------

MAX_TOTAL_DISTANCE_M = 0.10

MAX_CURRENT_ARM_VELOCITY = 0.10

MAX_GLOBAL_JOINT_DELTA_DEG = 18.0

TARGET_TOLERANCE_M = 0.010

MAX_RETURN_XYZ_ERROR_M = 0.020

# -------------------------------
# 关节跟踪误差补偿
# -------------------------------

MAX_CORRECTIONS = 2

# 每轮使用真实关节误差的50%
TRACKING_COMP_GAIN = 0.50

# 每轮单关节新增补偿最大1.5度
MAX_COMP_INCREMENT_DEG = 1.50

# 两轮累计补偿最大3度
MAX_COMP_TOTAL_DEG = 3.00

# 修正命令相对于当前真实关节，
# 单次最大允许运动7度
MAX_CORRECTION_MOVE_DEG = 7.0

# 补偿后的目标仍至少离机械限位3度
MIN_COMP_COMMAND_LIMIT_MARGIN_DEG = 3.0

# 补偿后理论末端允许最多越过目标3.5cm
MAX_COMP_PREDICTED_XYZ_OFFSET_M = 0.035

# 补偿后相对起始位置最多11cm
MAX_COMP_DISTANCE_FROM_START_M = 0.11

# position-only IK姿态安全限制
MAX_COMP_RPY_DELTA_DEG = 25.0

# 修正动作时间
CORRECTION_MOVE_SECONDS = 3.0
CORRECTION_HOLD_SECONDS = 2.5

# 若一次修正后误差改善不足1mm，
# 认为当前补偿基本没有效果
MIN_IMPROVEMENT_M = 0.001

# 返回原始关节姿态
RETURN_SECONDS = 6.0
RETURN_HOLD_SECONDS = 1.5

MAX_HAL_COMMAND_ERROR_RAD = 0.03


def right_limit_margins_deg(
    solver,
    arm,
):
    limits = (
        solver.joint_limits_for_arm_pos()
    )[7:]

    result = {}

    for q, (
        name,
        lower,
        upper,
    ) in zip(
        arm[7:],
        limits,
    ):
        margin_rad = min(
            float(q) - float(lower),
            float(upper) - float(q),
        )

        result[name] = math.degrees(
            margin_rad
        )

    return result


def validate_comp_command(
    solver,
    initial_arm,
    initial_head,
    actual_arm,
    command_arm,
    start_xyz,
    target_xyz,
):
    # --------------------------------
    # 1. 单次实际关节变化
    # --------------------------------

    move_deg = np.degrees(
        command_arm[7:]
        - actual_arm[7:]
    )

    max_move_idx = int(
        np.argmax(
            np.abs(move_deg)
        )
    )

    max_move_deg = float(
        abs(
            move_deg[
                max_move_idx
            ]
        )
    )

    max_move_joint = (
        ARM_POS_ORDER[
            7 + max_move_idx
        ]
    )

    print(
        "COMP_COMMAND_MAX_MOVE_DEG="
        f"{max_move_deg:.3f}"
    )

    print(
        "COMP_COMMAND_MAX_MOVE_JOINT="
        f"{max_move_joint}"
    )

    if (
        max_move_deg
        > MAX_CORRECTION_MOVE_DEG
    ):
        raise RuntimeError(
            "补偿命令单次关节运动过大："
            f"{max_move_deg:.3f}deg"
        )

    # --------------------------------
    # 2. 关节限位
    # --------------------------------

    margins = (
        right_limit_margins_deg(
            solver,
            command_arm,
        )
    )

    min_joint = min(
        margins,
        key=margins.get,
    )

    min_margin = (
        margins[min_joint]
    )

    print(
        "COMP_COMMAND_MIN_LIMIT_MARGIN_DEG="
        f"{min_margin:.3f}"
    )

    print(
        "COMP_COMMAND_MIN_LIMIT_MARGIN_JOINT="
        f"{min_joint}"
    )

    if (
        min_margin
        < MIN_COMP_COMMAND_LIMIT_MARGIN_DEG
    ):
        raise RuntimeError(
            "补偿命令过于接近机械限位："
            f"{min_joint}, "
            f"{min_margin:.3f}deg"
        )

    # --------------------------------
    # 3. 补偿命令理论FK
    # --------------------------------

    predicted_xyz = np.asarray(
        solver.fk_xyz(
            SIDE,
            command_arm,
            current_head_pos=initial_head,
        ),
        dtype=float,
    )

    predicted_target_offset = float(
        np.linalg.norm(
            predicted_xyz
            - target_xyz
        )
    )

    predicted_from_start = float(
        np.linalg.norm(
            predicted_xyz
            - start_xyz
        )
    )

    print(
        "COMP_COMMAND_PREDICTED_XYZ="
        f"{fmt_xyz(predicted_xyz)}"
    )

    print(
        "COMP_COMMAND_PREDICTED_TARGET_OFFSET_M="
        f"{predicted_target_offset:.6f}"
    )

    print(
        "COMP_COMMAND_PREDICTED_TARGET_OFFSET_CM="
        f"{predicted_target_offset * 100:.3f}"
    )

    print(
        "COMP_COMMAND_PREDICTED_FROM_START_CM="
        f"{predicted_from_start * 100:.3f}"
    )

    if (
        predicted_target_offset
        > MAX_COMP_PREDICTED_XYZ_OFFSET_M
    ):
        raise RuntimeError(
            "补偿后的理论末端位置"
            "偏离目标过大"
        )

    if (
        predicted_from_start
        > MAX_COMP_DISTANCE_FROM_START_M
    ):
        raise RuntimeError(
            "补偿后的理论末端"
            "距离起点超过11cm"
        )

    # --------------------------------
    # 4. RPY变化
    # --------------------------------

    start_rpy = np.asarray(
        solver.fk_rpy(
            SIDE,
            initial_arm,
            current_head_pos=initial_head,
        ),
        dtype=float,
    )

    predicted_rpy = np.asarray(
        solver.fk_rpy(
            SIDE,
            command_arm,
            current_head_pos=initial_head,
        ),
        dtype=float,
    )

    delta_rpy = np.asarray(
        [
            wrap_angle(
                predicted_rpy[i]
                - start_rpy[i]
            )
            for i in range(3)
        ]
    )

    max_rpy = float(
        np.max(
            np.abs(
                np.degrees(
                    delta_rpy
                )
            )
        )
    )

    print(
        "COMP_COMMAND_MAX_RPY_DELTA_DEG="
        f"{max_rpy:.3f}"
    )

    if (
        max_rpy
        > MAX_COMP_RPY_DELTA_DEG
    ):
        raise RuntimeError(
            "补偿后末端姿态变化过大："
            f"{max_rpy:.3f}deg"
        )

    return predicted_xyz


def validate_hal_command(
    node,
    command_arm,
):
    missing = [
        name
        for name in ARM_POS_ORDER
        if name not in node.latest_hal_arm
    ]

    if missing:
        raise RuntimeError(
            "HAL命令缺少关节："
            + ",".join(missing)
        )

    errors = []

    for name, q in zip(
        ARM_POS_ORDER,
        command_arm,
    ):
        errors.append(
            abs(
                node.latest_hal_arm[name]
                - float(q)
            )
        )

    max_error = float(
        max(errors)
    )

    print(
        "HAL_COMMAND_MAX_ERROR_RAD="
        f"{max_error:.9f}"
    )

    if (
        max_error
        > MAX_HAL_COMMAND_ERROR_RAD
    ):
        raise RuntimeError(
            "MC下发到HAL的目标"
            "与补偿命令不一致"
        )


def main():

    parser = argparse.ArgumentParser(
        description=(
            "X2比赛版右臂XYZ + "
            "真实关节跟踪误差补偿测试"
        )
    )

    group = (
        parser.add_mutually_exclusive_group(
            required=True
        )
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
    )

    parser.add_argument(
        "--execute",
        action="store_true",
    )

    args = parser.parse_args()

    rclpy.init()

    node = XYZClosedLoopNode()

    solver = X2ArmIKSolver(
        X2IKConfig.default_omnipicker()
    )

    initial_arm = None
    initial_head = None
    hand_pos = None

    switched_to_urs = False

    try:
        # ======================================
        # 1. 真实起点
        # ======================================

        (
            initial_arm,
            initial_head,
            velocity_container,
            start_xyz,
        ) = measure_robot(
            node,
            solver,
        )

        max_velocity = (
            extract_max_velocity(
                velocity_container
            )
        )

        print(
            "VELOCITY_CONTAINER_TYPE="
            f"{type(velocity_container).__name__}"
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
                "机械臂当前仍在明显运动"
            )

        hand_pos = (
            node.wait_for_claw_state()
        )

        if args.offset is not None:

            offset = np.asarray(
                args.offset,
                dtype=float,
            )

            target_xyz = (
                start_xyz
                + offset
            )

            mode = "OFFSET"

        else:

            target_xyz = np.asarray(
                args.target,
                dtype=float,
            )

            offset = (
                target_xyz
                - start_xyz
            )

            mode = "ABSOLUTE"

        distance = float(
            np.linalg.norm(offset)
        )

        if (
            distance
            > MAX_TOTAL_DISTANCE_M
        ):
            raise RuntimeError(
                "当前测试禁止目标距离超过10cm"
            )

        # ======================================
        # 2. 初始Cartesian waypoint规划
        # ======================================

        plan = plan_cartesian_path(
            solver,
            initial_arm,
            initial_head,
            start_xyz,
            target_xyz,
            waypoint_step_m=(
                WAYPOINT_STEP_M
            ),
            max_global_joint_delta_deg=(
                MAX_GLOBAL_JOINT_DELTA_DEG
            ),
            label="INITIAL_PLAN",
        )

        nominal_target_arm = np.asarray(
            plan["final_arm"],
            dtype=float,
        )

        # 左臂命令永远固定在起始真实位置
        nominal_target_arm[:7] = (
            initial_arm[:7]
        )

        print(
            "=== X2 XYZ真实关节跟踪补偿测试 ==="
        )

        print(
            f"EXECUTE={str(args.execute).lower()}"
        )

        print(
            f"TARGET_MODE={mode}"
        )

        print(
            f"START_XYZ={fmt_xyz(start_xyz)}"
        )

        print(
            f"TARGET_OFFSET_M={fmt_xyz(offset)}"
        )

        print(
            f"TARGET_XYZ={fmt_xyz(target_xyz)}"
        )

        print(
            "TARGET_TOLERANCE_M="
            f"{TARGET_TOLERANCE_M:.6f}"
        )

        print(
            "TRACKING_COMP_GAIN="
            f"{TRACKING_COMP_GAIN:.3f}"
        )

        print(
            "MAX_COMP_INCREMENT_DEG="
            f"{MAX_COMP_INCREMENT_DEG:.3f}"
        )

        print(
            "MAX_COMP_TOTAL_DEG="
            f"{MAX_COMP_TOTAL_DEG:.3f}"
        )

        print(
            "MAX_CORRECTIONS="
            f"{MAX_CORRECTIONS}"
        )

        print(
            "HAND_POS="
            + ",".join(
                f"{x:.6f}"
                for x in hand_pos
            )
        )

        print_plan(
            plan
        )

        if not args.execute:

            print(
                "REAL_ROBOT_MOTION=false"
            )

            print(
                "RESULT=PASS"
            )

            return 0

        # ======================================
        # 3. ROS图检查
        # ======================================

        subscribers = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0
            )
        )

        publishers = (
            node.count_publishers(
                COMMAND_TOPIC
            )
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command"
                "没有订阅者"
            )

        if publishers > 1:
            raise RuntimeError(
                "存在其他上肢command publisher"
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

        # ======================================
        # 4. URS接管
        # ======================================

        node.publish_segment(
            initial_arm,
            initial_arm,
            initial_head,
            hand_pos,
            PRE_URS_HOLD_SECONDS,
        )

        node.latest_hal_arm.clear()

        node.set_mode_while_holding(
            "UPPERBODY_REMOTE_SPLIT",
            initial_arm,
            initial_head,
            hand_pos,
        )

        switched_to_urs = True

        node.publish_segment(
            initial_arm,
            initial_arm,
            initial_head,
            hand_pos,
            POST_URS_HOLD_SECONDS,
        )

        node.validate_hal_takeover(
            initial_arm
        )

        # ======================================
        # 5. 初始Cartesian运动
        # ======================================

        print(
            "INITIAL_MOTION_START=true"
        )

        (
            frames,
            seconds,
            commanded_arm,
        ) = execute_plan(
            node,
            plan,
            initial_arm,
            initial_head,
            hand_pos,
            speed_mps=(
                CARTESIAN_SPEED_MPS
            ),
        )

        # 最终保持理论目标
        commanded_arm = (
            nominal_target_arm.copy()
        )

        node.publish_segment(
            commanded_arm,
            commanded_arm,
            initial_head,
            hand_pos,
            2.5,
        )

        print(
            f"INITIAL_MOTION_FRAMES={frames}"
        )

        print(
            "INITIAL_MOTION_SECONDS="
            f"{seconds:.3f}"
        )

        # ======================================
        # 6. 第一次实际测量
        # ======================================

        (
            actual_arm,
            actual_head,
            _,
            actual_xyz,
        ) = measure_robot(
            node,
            solver,
        )

        target_error = float(
            np.linalg.norm(
                target_xyz
                - actual_xyz
            )
        )

        print(
            f"PASS_0_REAL_XYZ={fmt_xyz(actual_xyz)}"
        )

        print(
            "PASS_0_TARGET_ERROR_M="
            f"{target_error:.6f}"
        )

        print(
            "PASS_0_TARGET_ERROR_CM="
            f"{target_error * 100:.3f}"
        )

        # 累计补偿
        compensation = np.zeros(
            7,
            dtype=float,
        )

        corrections_used = 0

        previous_error = (
            target_error
        )

        # ======================================
        # 7. 有限积分关节补偿
        # ======================================

        while (
            target_error
            > TARGET_TOLERANCE_M
            and corrections_used
            < MAX_CORRECTIONS
        ):

            corrections_used += 1

            tracking_error = (
                nominal_target_arm[7:]
                - actual_arm[7:]
            )

            tracking_error_deg = (
                np.degrees(
                    tracking_error
                )
            )

            print(
                f"CORRECTION_{corrections_used}_"
                "TRACKING_ERROR_DEG=["
                + ", ".join(
                    f"{x:+.3f}"
                    for x in tracking_error_deg
                )
                + "]"
            )

            increment = (
                TRACKING_COMP_GAIN
                * tracking_error
            )

            increment_cap = (
                math.radians(
                    MAX_COMP_INCREMENT_DEG
                )
            )

            increment = np.clip(
                increment,
                -increment_cap,
                +increment_cap,
            )

            compensation += increment

            total_cap = math.radians(
                MAX_COMP_TOTAL_DEG
            )

            compensation = np.clip(
                compensation,
                -total_cap,
                +total_cap,
            )

            print(
                f"CORRECTION_{corrections_used}_"
                "COMPENSATION_DEG=["
                + ", ".join(
                    f"{x:+.3f}"
                    for x in np.degrees(
                        compensation
                    )
                )
                + "]"
            )

            correction_command = (
                nominal_target_arm.copy()
            )

            correction_command[:7] = (
                initial_arm[:7]
            )

            correction_command[7:] += (
                compensation
            )

            print(
                "CORRECTION_START="
                f"{corrections_used}"
            )

            validate_comp_command(
                solver,
                initial_arm,
                initial_head,
                actual_arm,
                correction_command,
                start_xyz,
                target_xyz,
            )

            node.latest_hal_arm.clear()

            frames = node.publish_segment(
                actual_arm,
                correction_command,
                initial_head,
                hand_pos,
                CORRECTION_MOVE_SECONDS,
            )

            node.publish_segment(
                correction_command,
                correction_command,
                initial_head,
                hand_pos,
                CORRECTION_HOLD_SECONDS,
            )

            print(
                "CORRECTION_FRAMES="
                f"{frames}"
            )

            validate_hal_command(
                node,
                correction_command,
            )

            (
                actual_arm,
                actual_head,
                _,
                actual_xyz,
            ) = measure_robot(
                node,
                solver,
            )

            target_error = float(
                np.linalg.norm(
                    target_xyz
                    - actual_xyz
                )
            )

            improvement = (
                previous_error
                - target_error
            )

            print(
                f"PASS_{corrections_used}_"
                "REAL_XYZ="
                f"{fmt_xyz(actual_xyz)}"
            )

            print(
                f"PASS_{corrections_used}_"
                "TARGET_ERROR_M="
                f"{target_error:.6f}"
            )

            print(
                f"PASS_{corrections_used}_"
                "TARGET_ERROR_CM="
                f"{target_error * 100:.3f}"
            )

            print(
                f"PASS_{corrections_used}_"
                "IMPROVEMENT_M="
                f"{improvement:.6f}"
            )

            if (
                improvement
                < MIN_IMPROVEMENT_M
                and target_error
                > TARGET_TOLERANCE_M
            ):
                print(
                    "CORRECTION_STALLED=true"
                )
                break

            previous_error = (
                target_error
            )

        target_reached = (
            target_error
            <= TARGET_TOLERANCE_M
        )

        print(
            "TARGET_REACHED="
            f"{str(target_reached).lower()}"
        )

        print(
            "CORRECTIONS_USED="
            f"{corrections_used}"
        )

        print(
            "FINAL_TARGET_ERROR_M="
            f"{target_error:.6f}"
        )

        print(
            "FINAL_TARGET_ERROR_CM="
            f"{target_error * 100:.3f}"
        )

        # ======================================
        # 8. 安全返回
        # 不再使用返回IK。
        # ======================================

        print(
            "RETURN_TYPE=JOINT_SPACE_TO_REAL_START"
        )

        print(
            "RETURN_START=true"
        )

        (
            return_start_arm,
            return_start_head,
            _,
            return_start_xyz,
        ) = measure_robot(
            node,
            solver,
        )

        frames = node.publish_segment(
            return_start_arm,
            initial_arm,
            initial_head,
            hand_pos,
            RETURN_SECONDS,
        )

        print(
            f"RETURN_FRAMES={frames}"
        )

        node.publish_segment(
            initial_arm,
            initial_arm,
            initial_head,
            hand_pos,
            RETURN_HOLD_SECONDS,
        )

        (
            final_arm,
            final_head,
            _,
            final_xyz,
        ) = measure_robot(
            node,
            solver,
        )

        return_error = float(
            np.linalg.norm(
                final_xyz
                - start_xyz
            )
        )

        print(
            f"RETURN_REAL_XYZ={fmt_xyz(final_xyz)}"
        )

        print(
            "RETURN_XYZ_ERROR_M="
            f"{return_error:.6f}"
        )

        print(
            "RETURN_XYZ_ERROR_CM="
            f"{return_error * 100:.3f}"
        )

        node.set_mode_while_holding(
            "STAND_DEFAULT",
            initial_arm,
            initial_head,
            hand_pos,
        )

        switched_to_urs = False

        print(
            "FINAL_MODE=STAND_DEFAULT"
        )

        if not target_reached:

            print(
                "RESULT=FAIL"
            )

            print(
                "REASON=有限关节跟踪补偿后"
                "仍未进入1cm目标容差"
            )

            return 1

        if (
            return_error
            > MAX_RETURN_XYZ_ERROR_M
        ):

            print(
                "RESULT=FAIL"
            )

            print(
                "REASON=返回起始XYZ误差超过2cm"
            )

            return 1

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
            and initial_arm is not None
            and initial_head is not None
            and hand_pos is not None
        ):

            print(
                "RECOVERY=从当前真实关节"
                "缓慢返回最初真实关节"
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
                    initial_arm,
                    initial_head,
                    hand_pos,
                    5.0,
                )

                node.publish_segment(
                    initial_arm,
                    initial_arm,
                    initial_head,
                    hand_pos,
                    1.0,
                )

                node.set_mode_while_holding(
                    "STAND_DEFAULT",
                    initial_arm,
                    initial_head,
                    hand_pos,
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
