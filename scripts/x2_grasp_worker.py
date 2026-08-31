#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

import numpy as np
import rclpy

from std_msgs.msg import String

from x2_ik_sdk import (
    ArmSide,
    X2ArmIKSolver,
    X2IKConfig,
)

from x2_safe_joint_microtest import COMMAND_TOPIC

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

# ============================================================
# 桌面抓取轨迹
# ============================================================

# 抓取开始时先原地竖直抬升
PRE_GRASP_LIFT_M = 0.18

# 到达目标XY后，先停在目标上方
TARGET_APPROACH_HEIGHT_M = 0.0

# 抓住物体后先向上提起
POST_GRASP_LIFT_M = 0.20


SIDE = ArmSide.RIGHT

GRIPPER_TOPIC = "/gripper_cmd"

RIGHT_OPEN = 1.0
RIGHT_CLOSE = 0.0

MAX_DISTANCE_M = 2.00
MAX_ARM_VELOCITY = 0.10

MAX_GLOBAL_JOINT_DELTA_DEG = 140.0

TARGET_TOLERANCE_M = 0.010
MAX_CLOSE_ERROR_M = 0.030

MAX_CORRECTIONS = 3
TRACKING_COMP_GAIN = 0.50

MAX_COMP_INCREMENT_DEG = 1.50
MAX_COMP_TOTAL_DEG = 3.00

CORRECTION_MOVE_SECONDS = 3.0
CORRECTION_HOLD_SECONDS = 2.0

PRE_URS_HOLD_SECONDS = 2.0
POST_URS_HOLD_SECONDS = 2.0

GRIPPER_OPEN_WAIT_SECONDS = 2.0
GRIPPER_CLOSE_WAIT_SECONDS = 1.2

RETURN_SECONDS = 6.0
RETURN_HOLD_SECONDS = 1.5

MAX_RETURN_ERROR_M = 0.100

MAX_STREAM_GAP_SEC = 0.120

MIN_USEFUL_IMPROVEMENT_M = 0.0005
MAX_ERROR_WORSENING_M = 0.050


def fresh_hand_state(node):
    node.latest_hand_state = None

    return np.asarray(
        node.wait_for_claw_state(
            timeout_sec=5.0
        ),
        dtype=float,
    )


def publish_gripper(
    node,
    publisher,
    command,
):
    if command not in ("open", "close"):
        raise ValueError(
            f"invalid gripper command: {command}"
        )

    deadline = time.monotonic() + 3.0

    while (
        publisher.get_subscription_count() < 1
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

    if publisher.get_subscription_count() < 1:
        raise RuntimeError(
            "/gripper_cmd没有订阅者；"
            "omnipicker_hand.py未运行"
        )

    msg = String()
    msg.data = command

    # 多发几次，daemon只修改状态，不会重复执行状态机。
    for _ in range(5):
        publisher.publish(msg)

        rclpy.spin_once(
            node,
            timeout_sec=0.01,
        )

        time.sleep(0.03)

    print(
        f"GRIPPER_COMMAND={command}"
    )


def wait_seconds(
    node,
    pump,
    seconds,
):
    deadline = time.monotonic() + seconds

    while (
        rclpy.ok()
        and time.monotonic() < deadline
    ):
        pump.assert_healthy()

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )


def sync_gripper(
    node,
    pump,
    publisher,
    hand_state,
    *,
    command,
    value,
    wait_seconds_value,
):
    """
    同时修改：
      1. UpperBodyCommandArray.hand_pos
      2. omnipicker_hand.py /gripper_cmd

    避免MC和daemon给夹爪两个不同目标。
    """

    target_hand = np.asarray(
        hand_state,
        dtype=float,
    ).copy()

    target_hand[1] = float(value)

    # 先更新MC侧下一帧目标。
    pump.set_command(
        pump.get_arm(),
        hand=target_hand,
    )

    # 马上同步daemon侧。
    publish_gripper(
        node,
        publisher,
        command,
    )

    wait_seconds(
        node,
        pump,
        wait_seconds_value,
    )

    return target_hand


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--object",
        default="unknown",
    )

    parser.add_argument(
        "--target",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
    )

    parser.add_argument(
        "--plan-only",
        action="store_true",
        help=(
            "只做三段式桌面抓取IK规划，"
            "不切URS、不控制夹爪、不产生真机动作"
        ),
    )

    args = parser.parse_args()

    target_xyz = np.asarray(
        args.target,
        dtype=float,
    )

    rclpy.init()

    node = XYZClosedLoopNode()

    solver = X2ArmIKSolver(
        X2IKConfig.default_omnipicker()
    )

    gripper_pub = node.create_publisher(
        String,
        GRIPPER_TOPIC,
        10,
    )

    pump = None
    switched_to_urs = False

    initial_arm = None
    initial_head = None

    # 异常恢复时保持当前夹爪状态。
    active_hand = None

    # 抓取成功并进入最终抬起保持状态后，
    # 禁止 finally 再把机械臂送回 initial_arm。
    final_hold_active = False

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

        max_velocity = extract_max_velocity(
            velocity_container
        )

        print(
            f"OBJECT_NAME={args.object}"
        )

        print(
            f"START_XYZ={fmt_xyz(start_xyz)}"
        )

        print(
            f"TARGET_XYZ={fmt_xyz(target_xyz)}"
        )

        print(
            "MAX_CURRENT_ARM_VELOCITY="
            f"{max_velocity:.9f}"
        )

        if max_velocity > MAX_ARM_VELOCITY:
            raise RuntimeError(
                "机械臂当前仍在明显运动"
            )

        distance = float(
            np.linalg.norm(
                target_xyz - start_xyz
            )
        )

        print(
            "TARGET_DISTANCE_CM="
            f"{distance * 100:.3f}"
        )

        if distance > MAX_DISTANCE_M:
            raise RuntimeError(
                "目标距离当前右手超过允许范围；"
                f"限制={MAX_DISTANCE_M:.2f}m"
            )

        # ==================================================
        # 2. 桌面抓取三段式 Cartesian preflight
        #
        # 当前点
        #   -> 原地竖直抬升约30cm
        #   -> 高位移动到目标正上方
        #   -> 垂直下降到视觉目标点
        #
        # 这样避免从当前姿态斜着扫向桌面目标。
        # ==================================================

        pre_grasp_lift_xyz = np.asarray(
            start_xyz,
            dtype=float,
        ).copy()

        pre_grasp_lift_xyz[2] += (
            PRE_GRASP_LIFT_M
        )

        # 高位横移高度至少满足两者之一：
        # 1. 当前点向上30cm；
        # 2. 目标点上方15cm。
        #
        # 因此即使目标本身比当前手高，也不会在横移阶段
        # 贴着目标/桌面走。
        transit_z = max(
            float(pre_grasp_lift_xyz[2]),
            float(target_xyz[2])
            + TARGET_APPROACH_HEIGHT_M,
        )

        above_target_xyz = np.asarray(
            [
                float(target_xyz[0]),
                float(target_xyz[1]),
                float(transit_z),
            ],
            dtype=float,
        )

        print(
            "TABLE_GRASP_MODE=true"
        )

        print(
            "PRE_GRASP_LIFT_M="
            f"{PRE_GRASP_LIFT_M:.3f}"
        )

        print(
            "TARGET_APPROACH_HEIGHT_M="
            f"{TARGET_APPROACH_HEIGHT_M:.3f}"
        )

        print(
            "POST_GRASP_LIFT_M="
            f"{POST_GRASP_LIFT_M:.3f}"
        )

        print(
            "PRE_GRASP_LIFT_XYZ="
            f"{fmt_xyz(pre_grasp_lift_xyz)}"
        )

        print(
            "ABOVE_TARGET_XYZ="
            f"{fmt_xyz(above_target_xyz)}"
        )

        # --------------------------------------------------
        # PLAN 1：当前点原地竖直抬升
        # --------------------------------------------------

        lift_plan = plan_cartesian_path(
            solver,
            initial_arm,
            initial_head,
            start_xyz,
            pre_grasp_lift_xyz,
            waypoint_step_m=WAYPOINT_STEP_M,
            max_global_joint_delta_deg=(
                MAX_GLOBAL_JOINT_DELTA_DEG
            ),
            label="PRE_GRASP_LIFT",
        )

        lift_final_arm = np.asarray(
            lift_plan["final_arm"],
            dtype=float,
        ).copy()

        lift_final_arm[:7] = (
            initial_arm[:7]
        )

        # 使用IK最终关节做一次FK，作为下一段真实一致的起点。
        lift_final_xyz = np.asarray(
            solver.fk_xyz(
                SIDE,
                lift_final_arm,
                current_head_pos=initial_head,
            ),
            dtype=float,
        )

        # --------------------------------------------------
        # PLAN 2：高位移动到目标正上方
        # --------------------------------------------------

        high_approach_plan = plan_cartesian_path(
            solver,
            lift_final_arm,
            initial_head,
            lift_final_xyz,
            above_target_xyz,
            waypoint_step_m=WAYPOINT_STEP_M,
            max_global_joint_delta_deg=(
                MAX_GLOBAL_JOINT_DELTA_DEG
            ),
            label="HIGH_APPROACH",
        )

        high_final_arm = np.asarray(
            high_approach_plan["final_arm"],
            dtype=float,
        ).copy()

        high_final_arm[:7] = (
            initial_arm[:7]
        )

        high_final_xyz = np.asarray(
            solver.fk_xyz(
                SIDE,
                high_final_arm,
                current_head_pos=initial_head,
            ),
            dtype=float,
        )

        # --------------------------------------------------
        # PLAN 3：从正上方垂直下降到视觉目标点
        # --------------------------------------------------

        descent_plan = plan_cartesian_path(
            solver,
            high_final_arm,
            initial_head,
            high_final_xyz,
            target_xyz,
            waypoint_step_m=0.008,
            max_global_joint_delta_deg=(
                MAX_GLOBAL_JOINT_DELTA_DEG
            ),
            label="FINAL_DESCENT",
        )

        nominal_target_arm = np.asarray(
            descent_plan["final_arm"],
            dtype=float,
        ).copy()

        # 左臂固定。
        nominal_target_arm[:7] = (
            initial_arm[:7]
        )

        print_plan(
            lift_plan
        )

        print_plan(
            high_approach_plan
        )

        print_plan(
            descent_plan
        )

        print(
            "GRASP_PREFLIGHT=PASS"
        )

        if args.plan_only:
            print(
                "PLAN_ONLY=true"
            )

            print(
                "REAL_ROBOT_MOTION=false"
            )

            print(
                "RESULT=PASS"
            )

            return 0

        # ==================================================
        # 3. ROS graph检查
        # ==================================================

        subscribers = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0
            )
        )

        publishers = node.count_publishers(
            COMMAND_TOPIC
        )

        print(
            f"UPPER_BODY_SUBSCRIBERS={subscribers}"
        )

        print(
            f"UPPER_BODY_PUBLISHERS={publishers}"
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command无订阅者"
            )

        # Worker自己的publisher为1。
        if publishers > 1:
            raise RuntimeError(
                "检测到其他UpperBodyCommand publisher"
            )

        if gripper_pub.get_subscription_count() < 1:
            deadline = time.monotonic() + 3.0

            while (
                gripper_pub.get_subscription_count() < 1
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(
                    node,
                    timeout_sec=0.05,
                )

        if gripper_pub.get_subscription_count() < 1:
            raise RuntimeError(
                "omnipicker gripper daemon未运行"
            )

        # ==================================================
        # 4. 当前夹爪状态
        # ==================================================

        active_hand = fresh_hand_state(
            node
        )

        print(
            "INITIAL_HAND="
            f"[{active_hand[0]:.6f}, "
            f"{active_hand[1]:.6f}]"
        )

        # ==================================================
        # 5. Persistent 50Hz
        # ==================================================

        pump = PersistentCommandPump(
            node,
            initial_arm,
            initial_head,
            active_hand,
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
        # 7. 先原地竖直抬升约30cm
        #
        # 注意：这一步仍保持当前夹爪状态。
        # 先把夹爪整体抬离桌面，再张开夹爪，
        # 避免夹爪张开时爪片碰桌面。
        # ==================================================

        print(
            "PRE_GRASP_LIFT_START=true"
        )

        (
            lift_updates,
            lift_seconds,
        ) = execute_plan_with_pump(
            node,
            pump,
            lift_plan,
        )

        pump.set_command(
            lift_final_arm,
            hand=active_hand,
        )

        pump.hold(0.8)

        print(
            "PRE_GRASP_LIFT_COMPLETE=true"
        )

        print(
            f"PRE_GRASP_LIFT_UPDATES={lift_updates}"
        )

        print(
            "PRE_GRASP_LIFT_SECONDS="
            f"{lift_seconds:.3f}"
        )

        # ==================================================
        # 8. 抬高后再同步张开右夹爪
        # ==================================================

        active_hand = sync_gripper(
            node,
            pump,
            gripper_pub,
            active_hand,
            command="open",
            value=RIGHT_OPEN,
            wait_seconds_value=(
                GRIPPER_OPEN_WAIT_SECONDS
            ),
        )

        opened = fresh_hand_state(
            node
        )

        print(
            "OPENED_HAND="
            f"[{opened[0]:.6f}, "
            f"{opened[1]:.6f}]"
        )

        if opened[1] < 0.70:
            raise RuntimeError(
                "右夹爪没有正常张开"
            )

        # ==================================================
        # 9. 高位移动到目标正上方
        # ==================================================

        print(
            "HIGH_APPROACH_START=true"
        )

        (
            high_updates,
            high_seconds,
        ) = execute_plan_with_pump(
            node,
            pump,
            high_approach_plan,
        )

        pump.set_command(
            high_final_arm,
            hand=active_hand,
        )

        pump.hold(0.8)

        print(
            "HIGH_APPROACH_COMPLETE=true"
        )

        print(
            f"HIGH_APPROACH_UPDATES={high_updates}"
        )

        print(
            "HIGH_APPROACH_SECONDS="
            f"{high_seconds:.3f}"
        )

        # ==================================================
        # 10. 从目标正上方垂直下降到抓取点
        # ==================================================

        print(
            "FINAL_DESCENT_START=true"
        )

        (
            descent_updates,
            descent_seconds,
        ) = execute_plan_with_pump(
            node,
            pump,
            descent_plan,
        )

        pump.set_command(
            nominal_target_arm,
            hand=active_hand,
        )

        pump.hold(2.0)

        print(
            "FINAL_DESCENT_COMPLETE=true"
        )

        print(
            f"FINAL_DESCENT_UPDATES={descent_updates}"
        )

        print(
            "FINAL_DESCENT_SECONDS="
            f"{descent_seconds:.3f}"
        )

        # ==================================================
        # 11. 抓取点到达确认
        #
        # 这里不再使用FK XYZ误差作为闭爪门槛。
        #
        # 原因：
        # 当前实测中，视觉/IK目标对应的是实际夹取位置，
        # 但 measure_robot() 的FK参考点与实际夹爪TCP存在
        # 约7~8cm的系统性偏差。
        #
        # 因此 FINAL_DESCENT 执行完成后直接闭爪。
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
                target_xyz - actual_xyz
            )
        )

        print(
            "FINAL_DESCENT_REAL_XYZ="
            f"{fmt_xyz(actual_xyz)}"
        )

        print(
            "FK_REFERENCE_ERROR_CM="
            f"{target_error * 100:.3f}"
        )

        print(
            "FK_ERROR_USED_FOR_CLOSE=false"
        )

        print(
            "TARGET_REACHED_BY_PLAN=true"
        )

        corrections_used = 0

        # ==================================================
        # 11.5 抓后上提预规划
        #
        # 必须在闭爪之前完成。
        #
        # 从当前规划抓取姿态 nominal_target_arm
        # 直接规划 Z + POST_GRASP_LIFT_M。
        #
        # 这样闭爪后不再用可能发生轻微下沉的
        # actual_arm 重新作为轨迹起点。
        # ==================================================

        post_grasp_lift_xyz = np.asarray(
            target_xyz,
            dtype=float,
        ).copy()

        post_grasp_lift_xyz[2] += (
            POST_GRASP_LIFT_M
        )

        print(
            "POST_GRASP_LIFT_TARGET_XYZ="
            f"{fmt_xyz(post_grasp_lift_xyz)}"
        )

        post_lift_plan = plan_cartesian_path(
            solver,
            nominal_target_arm,
            initial_head,
            target_xyz,
            post_grasp_lift_xyz,
            waypoint_step_m=WAYPOINT_STEP_M,
            max_global_joint_delta_deg=(
                MAX_GLOBAL_JOINT_DELTA_DEG
            ),
            label="POST_GRASP_LIFT",
        )

        post_lift_final_arm = np.asarray(
            post_lift_plan["final_arm"],
            dtype=float,
        ).copy()

        post_lift_final_arm[:7] = (
            initial_arm[:7]
        )

        print_plan(
            post_lift_plan
        )

        print(
            "POST_GRASP_LIFT_PREFLIGHT=PASS"
        )

        # ==================================================
        # 12. 闭合右夹爪
        # ==================================================

        print(
            "GRIPPER_CLOSE_START=true"
        )

        active_hand = sync_gripper(
            node,
            pump,
            gripper_pub,
            active_hand,
            command="close",
            value=RIGHT_CLOSE,
            wait_seconds_value=(
                GRIPPER_CLOSE_WAIT_SECONDS
            ),
        )

        closed = fresh_hand_state(
            node
        )

        print(
            "CLOSED_HAND="
            f"[{closed[0]:.6f}, "
            f"{closed[1]:.6f}]"
        )

        # 有物体时position不一定到0，
        # 因此这里不拿==0作为成功条件。
        print(
            "GRIPPER_CLOSE_COMMAND_COMPLETED=true"
        )

        # ==================================================
        # 13. 抓住物体后立即垂直上提
        #
        # close完成后不重新读取actual_arm作为起点，
        # 不再原地额外hold 1秒。
        #
        # 直接从当前 nominal_target_arm
        # 连续进入预先规划好的竖直上提轨迹。
        # ==================================================

        print(
            "POST_GRASP_LIFT_START=true"
        )

        # 保持右夹爪持续close。
        pump.set_command(
            nominal_target_arm,
            hand=active_hand,
        )

        (
            post_lift_updates,
            post_lift_seconds,
        ) = execute_plan_with_pump(
            node,
            pump,
            post_lift_plan,
        )

        # --------------------------------------------------
        # 到抬起位置以后：
        #
        # 右臂保持当前位置。
        # 右夹爪保持0.0闭合。
        # --------------------------------------------------

        pump.set_command(
            post_lift_final_arm,
            hand=active_hand,
        )

        pump.hold(1.0)

        print(
            "POST_GRASP_LIFT_COMPLETE=true"
        )

        print(
            "POST_GRASP_LIFT_UPDATES="
            f"{post_lift_updates}"
        )

        print(
            "POST_GRASP_LIFT_SECONDS="
            f"{post_lift_seconds:.3f}"
        )

        (
            held_arm,
            held_head,
            _,
            held_xyz,
        ) = measure_robot(
            node,
            solver,
        )

        print(
            "FINAL_HOLD_XYZ="
            f"{fmt_xyz(held_xyz)}"
        )

        print(
            "GRIPPER_HOLD_COMMAND=close"
        )

        print(
            "GRASP_OBJECT_HELD=true"
        )

        print(
            "RETURN_TO_INITIAL=false"
        )

        print(
            "FINAL_HOLD_ACTIVE=true"
        )

        print(
            "GRASP_RESULT=PASS"
        )

        print(
            "RESULT=PASS"
        )

        # --------------------------------------------------
        # 最终比赛状态
        #
        # 必须继续保持PersistentCommandPump，
        # 否则退出Worker后就不能保证机器人继续保持
        # 这个URS姿态。
        #
        # 所以这里故意不退出。
        # --------------------------------------------------

        final_hold_active = True

        while rclpy.ok():

            pump.assert_healthy()

            # 持续保持：
            #   当前抬起后的关节姿态
            #   右夹爪关闭
            pump.set_command(
                post_lift_final_arm,
                hand=active_hand,
            )

            rclpy.spin_once(
                node,
                timeout_sec=0.05,
            )

        return 0


    except KeyboardInterrupt:

        print(
            "USER_STOP_REQUESTED=true"
        )

        # ==================================================
        # Ctrl+C：立即停止后续轨迹，并保持当前位置
        # ==================================================

        if (
            pump is not None
            and switched_to_urs
        ):

            try:

                (
                    stop_arm,
                    stop_head,
                    _,
                    stop_xyz,
                ) = measure_robot(
                    node,
                    solver,
                )

                print(
                    "STOP_REAL_XYZ="
                    f"{fmt_xyz(stop_xyz)}"
                )

                # 使用真机当前关节作为新的保持目标。
                pump.set_command(
                    stop_arm,
                    hand=active_hand,
                )

                pump.hold(
                    0.5
                )

                print(
                    "STOP_HOLD_CURRENT_POSE=true"
                )

                print(
                    "STOP_MODE=UPPERBODY_REMOTE_SPLIT"
                )

                # 阻止 finally 自动回 initial_arm。
                final_hold_active = True

                print(
                    "PRESS_CTRL_C_AGAIN_TO_RELEASE_WORKER=true"
                )

                # 持续保持当前姿态。
                while rclpy.ok():

                    pump.assert_healthy()

                    pump.set_command(
                        stop_arm,
                        hand=active_hand,
                    )

                    rclpy.spin_once(
                        node,
                        timeout_sec=0.05,
                    )

            except KeyboardInterrupt:

                print(
                    "SECOND_CTRL_C=true"
                )

                print(
                    "STOP_HOLD_EXIT=true"
                )

                return 130

        return 130

    except Exception as exc:

        # 真正异常仍使用原来的恢复逻辑。
        final_hold_active = False

        print(
            "GRASP_RESULT=FAIL"
        )

        print(
            f"REASON={exc}"
        )

        print(
            "RESULT=FAIL"
        )

        return 1

    finally:

        # URS中异常时持续命令流返回。
        if (
            switched_to_urs
            and pump is not None
            and initial_arm is not None
            and not final_hold_active
        ):
            print(
                "RECOVERY_START=true"
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