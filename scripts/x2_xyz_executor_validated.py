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
    COMMAND_TOPIC,
)

from x2_competition_move_xyz_closed_loop import (
    XYZClosedLoopNode,
    measure_robot,
    plan_cartesian_path,
    print_plan,
    extract_max_velocity,
    fmt_xyz,
    WAYPOINT_STEP_M,
)

from x2_competition_xyz_persistent_stream_test import (
    PersistentCommandPump,
    set_mode_with_pump,
    execute_plan_with_pump,
)

from x2_competition_move_xyz_tracking_comp import (
    validate_comp_command,
    validate_hal_command,
)


SIDE = ArmSide.RIGHT

MAX_TOTAL_DISTANCE_M = 0.10
MAX_CURRENT_ARM_VELOCITY = 0.10

MAX_GLOBAL_JOINT_DELTA_DEG = 18.0

TARGET_TOLERANCE_M = 0.010

MAX_CORRECTIONS = 3

TRACKING_COMP_GAIN = 0.50

MAX_COMP_INCREMENT_DEG = 1.50
MAX_COMP_TOTAL_DEG = 3.00

CORRECTION_MOVE_SECONDS = 3.0
CORRECTION_HOLD_SECONDS = 2.5

RETURN_SECONDS = 6.0
RETURN_HOLD_SECONDS = 1.5

MAX_RETURN_ERROR_M = 0.020

PRE_URS_HOLD_SECONDS = 2.0
POST_URS_HOLD_SECONDS = 2.0

# 如果修正反而让误差恶化超过3mm，
# 直接停止继续追。
MAX_ALLOWED_ERROR_WORSENING_M = 0.003

# 如果改善不足0.5mm，
# 认为修正基本进入平台期。
MIN_USEFUL_IMPROVEMENT_M = 0.0005

MAX_STREAM_GAP_SEC = 0.120


def main():

    parser = argparse.ArgumentParser(
        description=(
            "X2比赛版右臂"
            "持续50Hz + XYZ跟踪补偿测试"
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
        metavar=("DX", "DY", "DZ"),
    )

    group.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
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

    pump = None
    switched_to_urs = False

    try:

        # ==================================================
        # 1. 当前真实状态
        # ==================================================

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

        # ==================================================
        # 2. 目标
        # ==================================================

        if args.offset is not None:

            offset = np.asarray(
                args.offset,
                dtype=float,
            )

            target_xyz = (
                start_xyz
                + offset
            )

            target_mode = "OFFSET"

        else:

            target_xyz = np.asarray(
                args.target,
                dtype=float,
            )

            offset = (
                target_xyz
                - start_xyz
            )

            target_mode = "ABSOLUTE"

        distance = float(
            np.linalg.norm(offset)
        )

        if distance > MAX_TOTAL_DISTANCE_M:
            raise RuntimeError(
                "当前测试禁止一次移动超过10cm"
            )

        # ==================================================
        # 3. Cartesian路径离线规划
        # ==================================================

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
        ).copy()

        # 永远固定左臂
        nominal_target_arm[:7] = (
            initial_arm[:7]
        )

        print(
            "=== X2持续50Hz + XYZ跟踪补偿测试 ==="
        )

        print(
            f"EXECUTE={str(args.execute).lower()}"
        )

        print(
            f"TARGET_MODE={target_mode}"
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
            f"TARGET_TOLERANCE_M={TARGET_TOLERANCE_M:.6f}"
        )

        print(
            f"TRACKING_COMP_GAIN={TRACKING_COMP_GAIN:.3f}"
        )

        print(
            f"MAX_CORRECTIONS={MAX_CORRECTIONS}"
        )

        print(
            "COMMAND_STREAM_RATE_HZ=50"
        )

        print(
            "COMMAND_STREAM_POLICY="
            "HOLD_LAST_COMMAND_CONTINUOUSLY"
        )

        print_plan(plan)

        if not args.execute:

            print(
                "REAL_ROBOT_MOTION=false"
            )

            print(
                "RESULT=PASS"
            )

            return 0

        # ==================================================
        # 4. ROS图检查
        # ==================================================

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
                "/mc/upper_body_command无订阅者"
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

        # ==================================================
        # 5. 启动永久50Hz command pump
        # ==================================================

        pump = PersistentCommandPump(
            node,
            initial_arm,
            initial_head,
            hand_pos,
        )

        pump.start()

        pump.hold(
            PRE_URS_HOLD_SECONDS
        )

        print(
            "COMMAND_PUMP_STARTED=true"
        )

        # ==================================================
        # 6. 进入URS
        # ==================================================

        set_mode_with_pump(
            node,
            pump,
            "UPPERBODY_REMOTE_SPLIT",
        )

        switched_to_urs = True

        pump.hold(
            POST_URS_HOLD_SECONDS
        )

        node.validate_hal_takeover(
            initial_arm
        )

        # ==================================================
        # 7. 第一轮Cartesian运动
        # ==================================================

        print(
            "INITIAL_MOTION_START=true"
        )

        (
            updates,
            seconds,
        ) = execute_plan_with_pump(
            node,
            pump,
            plan,
        )

        # 确保最后持续保持理论IK终点
        pump.set_command(
            nominal_target_arm
        )

        pump.hold(2.5)

        print(
            f"INITIAL_MOTION_UPDATES={updates}"
        )

        print(
            f"INITIAL_MOTION_SECONDS={seconds:.3f}"
        )

        # ==================================================
        # 8. 第一轮真实状态
        # command pump在整个measure期间不会停
        # ==================================================

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

        # ==================================================
        # 9. 跟踪误差积分补偿
        # ==================================================

        compensation = np.zeros(
            7,
            dtype=float,
        )

        previous_error = (
            target_error
        )

        corrections_used = 0

        for correction_id in range(
            1,
            MAX_CORRECTIONS + 1,
        ):

            if (
                target_error
                <= TARGET_TOLERANCE_M
            ):
                break

            corrections_used = (
                correction_id
            )

            # ----------------------------------------------
            # 理论目标关节 - 当前真实关节
            # ----------------------------------------------

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
                f"CORRECTION_{correction_id}_"
                "TRACKING_ERROR_DEG=["
                + ", ".join(
                    f"{x:+.3f}"
                    for x in tracking_error_deg
                )
                + "]"
            )

            # ----------------------------------------------
            # 本轮新增补偿
            # ----------------------------------------------

            increment = (
                TRACKING_COMP_GAIN
                * tracking_error
            )

            increment_limit = (
                math.radians(
                    MAX_COMP_INCREMENT_DEG
                )
            )

            increment = np.clip(
                increment,
                -increment_limit,
                +increment_limit,
            )

            compensation += (
                increment
            )

            total_limit = (
                math.radians(
                    MAX_COMP_TOTAL_DEG
                )
            )

            compensation = np.clip(
                compensation,
                -total_limit,
                +total_limit,
            )

            print(
                f"CORRECTION_{correction_id}_"
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
                f"CORRECTION_START={correction_id}"
            )

            # ----------------------------------------------
            # 真机前补偿命令安全检查
            # ----------------------------------------------

            validate_comp_command(
                solver,
                initial_arm,
                initial_head,
                actual_arm,
                correction_command,
                start_xyz,
                target_xyz,
            )

            # ----------------------------------------------
            # 持续流下移动
            # ----------------------------------------------

            node.latest_hal_arm.clear()

            pump.move_to(
                correction_command,
                CORRECTION_MOVE_SECONDS,
            )

            pump.hold(
                CORRECTION_HOLD_SECONDS
            )

            validate_hal_command(
                node,
                correction_command,
            )

            # ----------------------------------------------
            # 再读取真实位置
            # 注意pump仍在发
            # ----------------------------------------------

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
                f"PASS_{correction_id}_REAL_XYZ="
                f"{fmt_xyz(actual_xyz)}"
            )

            print(
                f"PASS_{correction_id}_TARGET_ERROR_M="
                f"{target_error:.6f}"
            )

            print(
                f"PASS_{correction_id}_TARGET_ERROR_CM="
                f"{target_error * 100:.3f}"
            )

            print(
                f"PASS_{correction_id}_IMPROVEMENT_M="
                f"{improvement:.6f}"
            )

            # ----------------------------------------------
            # 如果明显反向恶化，停止继续补偿
            # ----------------------------------------------

            if (
                target_error
                - previous_error
                > MAX_ALLOWED_ERROR_WORSENING_M
            ):
                print(
                    "CORRECTION_ABORT=true"
                )

                print(
                    "CORRECTION_ABORT_REASON="
                    "误差明显恶化"
                )

                break

            # ----------------------------------------------
            # 如果进入平台期，也不继续盲目追
            # ----------------------------------------------

            if (
                improvement
                < MIN_USEFUL_IMPROVEMENT_M
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

        # ==================================================
        # 10. 最终结果
        # ==================================================

        target_reached = (
            target_error
            <= TARGET_TOLERANCE_M
        )

        print(
            "TARGET_REACHED="
            f"{str(target_reached).lower()}"
        )

        print(
            f"CORRECTIONS_USED={corrections_used}"
        )

        print(
            "FINAL_TARGET_ERROR_M="
            f"{target_error:.6f}"
        )

        print(
            "FINAL_TARGET_ERROR_CM="
            f"{target_error * 100:.3f}"
        )

        # ==================================================
        # 11. 正常返回
        # 这里也是持续50Hz，不再断流
        # ==================================================

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

        pump.move_to(
            initial_arm,
            RETURN_SECONDS,
        )

        pump.hold(
            RETURN_HOLD_SECONDS
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

        # ==================================================
        # 12. 先退出URS，再停止command pump
        # ==================================================

        set_mode_with_pump(
            node,
            pump,
            "STAND_DEFAULT",
        )

        switched_to_urs = False

        print(
            "FINAL_MODE=STAND_DEFAULT"
        )

        pump.assert_healthy()

        print(
            "COMMAND_PUMP_PUBLISH_COUNT="
            f"{pump.publish_count}"
        )

        print(
            "COMMAND_PUMP_MAX_GAP_MS="
            f"{pump.max_gap_sec * 1000:.3f}"
        )

        if (
            pump.max_gap_sec
            > MAX_STREAM_GAP_SEC
        ):
            raise RuntimeError(
                "URS期间命令流出现超过120ms中断"
            )

        print(
            "COMMAND_STREAM_CONTINUOUS=true"
        )

        if not target_reached:

            print(
                "RESULT=FAIL"
            )

            print(
                "REASON=两轮有限补偿后"
                "仍未进入1cm容差"
            )

            return 1

        if (
            return_error
            > MAX_RETURN_ERROR_M
        ):

            print(
                "RESULT=FAIL"
            )

            print(
                "REASON=返回起点误差超过2cm"
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
            and pump is not None
            and initial_arm is not None
        ):

            print(
                "RECOVERY=持续50Hz下返回初始姿态"
            )

            try:

                pump.move_to(
                    initial_arm,
                    5.0,
                )

                pump.hold(1.0)

                set_mode_with_pump(
                    node,
                    pump,
                    "STAND_DEFAULT",
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

        if pump is not None:

            pump.stop()

            print(
                "COMMAND_PUMP_STOPPED=true"
            )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
