#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy

from aimdk_msgs.msg import (
    CommonState,
    McActionCommand,
    RequestHeader,
)
from aimdk_msgs.srv import SetMcAction

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


SIDE = ArmSide.RIGHT

PUBLISH_RATE_HZ = 50.0
PUBLISH_PERIOD = 1.0 / PUBLISH_RATE_HZ

CARTESIAN_SPEED_MPS = 0.0125
MIN_SEGMENT_SECONDS = 0.35

PRE_URS_HOLD_SECONDS = 2.0

# 到达以后连续保持5秒，
# 中间测两次状态，确认不会“突然返回”
TARGET_HOLD_1_SECONDS = 1.5
TARGET_HOLD_2_SECONDS = 3.5

RETURN_SECONDS = 6.0
RETURN_HOLD_SECONDS = 1.5

MAX_CURRENT_ARM_VELOCITY = 0.10
MAX_TOTAL_DISTANCE_M = 0.10
MAX_GLOBAL_JOINT_DELTA_DEG = 18.0

# 命令泵如果出现超过120ms无发布，认为异常
MAX_ALLOWED_PUBLISH_GAP_SEC = 0.120


class PersistentCommandPump:

    def __init__(
        self,
        node,
        arm,
        head,
        hand,
    ):
        self.node = node

        self._lock = threading.Lock()

        self._arm = np.asarray(
            arm,
            dtype=float,
        ).copy()

        self._head = np.asarray(
            head,
            dtype=float,
        ).copy()

        self._hand = np.asarray(
            hand,
            dtype=float,
        ).copy()

        self._stop_event = (
            threading.Event()
        )

        self._thread = None
        self._exception = None

        self._publish_count = 0

        self._last_publish_time = None
        self._max_gap = 0.0

    def start(self):
        if self._thread is not None:
            raise RuntimeError(
                "command pump已经启动"
            )

        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="x2-upper-body-command-pump",
        )

        self._thread.start()

    def stop(self):
        self._stop_event.set()

        if self._thread is not None:
            self._thread.join(
                timeout=2.0
            )

    def _run(self):
        try:
            next_tick = time.monotonic()

            while (
                not self._stop_event.is_set()
                and rclpy.ok()
            ):
                with self._lock:
                    arm = self._arm.copy()
                    head = self._head.copy()
                    hand = self._hand.copy()

                now = time.monotonic()

                if (
                    self._last_publish_time
                    is not None
                ):
                    gap = (
                        now
                        - self._last_publish_time
                    )

                    if gap > self._max_gap:
                        self._max_gap = gap

                self._last_publish_time = now

                # 整个URS期间唯一的上肢command发布入口
                self.node.publish_upper(
                    arm,
                    head,
                    hand,
                )

                self._publish_count += 1

                next_tick += PUBLISH_PERIOD

                delay = (
                    next_tick
                    - time.monotonic()
                )

                if delay > 0:
                    time.sleep(delay)
                else:
                    # 如果线程偶尔落后，不累计漂移
                    next_tick = time.monotonic()

        except Exception as exc:
            self._exception = exc

    def assert_healthy(self):
        if self._exception is not None:
            raise RuntimeError(
                "command pump异常："
                f"{self._exception}"
            )

        if (
            self._thread is None
            or not self._thread.is_alive()
        ):
            raise RuntimeError(
                "command pump没有运行"
            )

    def set_command(
        self,
        arm,
        head=None,
        hand=None,
    ):
        with self._lock:
            self._arm = np.asarray(
                arm,
                dtype=float,
            ).copy()

            if head is not None:
                self._head = np.asarray(
                    head,
                    dtype=float,
                ).copy()

            if hand is not None:
                self._hand = np.asarray(
                    hand,
                    dtype=float,
                ).copy()

    def get_arm(self):
        with self._lock:
            return self._arm.copy()

    def move_to(
        self,
        target_arm,
        duration,
    ):
        self.assert_healthy()

        target_arm = np.asarray(
            target_arm,
            dtype=float,
        )

        start_arm = self.get_arm()

        start_time = time.monotonic()
        deadline = (
            start_time
            + duration
        )

        updates = 0

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):
            elapsed = (
                time.monotonic()
                - start_time
            )

            alpha = min(
                1.0,
                max(
                    0.0,
                    elapsed / duration,
                ),
            )

            q = (
                start_arm
                + (
                    target_arm
                    - start_arm
                )
                * alpha
            )

            self.set_command(q)

            # 主线程只负责处理ROS回调，
            # 真正publish由后台pump负责。
            rclpy.spin_once(
                self.node,
                timeout_sec=0.001,
            )

            updates += 1

            time.sleep(
                PUBLISH_PERIOD
            )

        self.set_command(
            target_arm
        )

        return updates

    def hold(
        self,
        seconds,
    ):
        self.assert_healthy()

        deadline = (
            time.monotonic()
            + seconds
        )

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):
            rclpy.spin_once(
                self.node,
                timeout_sec=0.02,
            )

        self.assert_healthy()

    @property
    def publish_count(self):
        return self._publish_count

    @property
    def max_gap_sec(self):
        return self._max_gap


def set_mode_with_pump(
    node,
    pump,
    action_name,
):
    if not node.mode_client.wait_for_service(
        timeout_sec=5.0
    ):
        raise RuntimeError(
            "SetMcAction服务不可用"
        )

    req = SetMcAction.Request()

    req.header = RequestHeader()
    req.header.stamp = (
        node.get_clock().now().to_msg()
    )

    req.source = "node.set_mc_action"

    cmd = McActionCommand()
    cmd.action_desc = action_name

    req.command = cmd

    print(
        f"MODE_REQUEST={action_name}"
    )

    future = (
        node.mode_client.call_async(req)
    )

    deadline = (
        time.monotonic()
        + 5.0
    )

    # 注意：
    # 等服务响应期间command pump仍在后台50Hz发布。
    while (
        rclpy.ok()
        and not future.done()
        and time.monotonic()
        < deadline
    ):
        pump.assert_healthy()

        rclpy.spin_once(
            node,
            timeout_sec=0.02,
        )

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

    if (
        status
        != int(CommonState.SUCCESS)
    ):
        raise RuntimeError(
            f"模式切换失败：{action_name}"
        )

    print(
        f"MODE_SET_SUCCESS={action_name}"
    )


def execute_plan_with_pump(
    node,
    pump,
    plan,
):
    total_updates = 0
    total_seconds = 0.0

    for wp in plan["waypoints"]:

        duration = max(
            MIN_SEGMENT_SECONDS,
            float(
                wp["xyz_step"]
            )
            / CARTESIAN_SPEED_MPS,
        )

        updates = pump.move_to(
            wp["arm"],
            duration,
        )

        total_updates += updates
        total_seconds += duration

    return (
        total_updates,
        total_seconds,
    )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--offset",
        nargs=3,
        type=float,
        required=True,
        metavar=(
            "DX",
            "DY",
            "DZ",
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

    pump = None
    switched_to_urs = False

    try:
        # ==================================
        # 1. 当前真实状态
        # ==================================

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

        offset = np.asarray(
            args.offset,
            dtype=float,
        )

        target_xyz = (
            start_xyz
            + offset
        )

        distance = float(
            np.linalg.norm(offset)
        )

        if (
            distance
            > MAX_TOTAL_DISTANCE_M
        ):
            raise RuntimeError(
                "测试目标距离超过10cm"
            )

        # ==================================
        # 2. 全路径先离线规划
        # ==================================

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

        print(
            "=== X2持续50Hz XYZ控制流测试 ==="
        )

        print(
            f"EXECUTE={str(args.execute).lower()}"
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

        # ==================================
        # 3. ROS graph
        # ==================================

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
                "发现其他上肢command发布器"
            )

        print(
            f"COMMAND_SUBSCRIBERS={subscribers}"
        )

        print(
            f"COMMAND_PUBLISHERS={publishers}"
        )

        # ==================================
        # 4. 启动永久command pump
        # ==================================

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

        # ==================================
        # 5. URS
        # ==================================

        set_mode_with_pump(
            node,
            pump,
            "UPPERBODY_REMOTE_SPLIT",
        )

        switched_to_urs = True

        pump.hold(2.0)

        node.validate_hal_takeover(
            initial_arm
        )

        # ==================================
        # 6. 沿Cartesian waypoint移动
        # ==================================

        print(
            "XYZ_MOTION_START=true"
        )

        (
            updates,
            seconds,
        ) = execute_plan_with_pump(
            node,
            pump,
            plan,
        )

        print(
            f"MOTION_UPDATES={updates}"
        )

        print(
            f"MOTION_SECONDS={seconds:.3f}"
        )

        # ==================================
        # 7. 到终点后持续保持。
        #
        # 重点：在measure_robot()过程中
        # command pump不会停。
        # ==================================

        pump.hold(
            TARGET_HOLD_1_SECONDS
        )

        (
            arm1,
            head1,
            _,
            xyz1,
        ) = measure_robot(
            node,
            solver,
        )

        error1 = float(
            np.linalg.norm(
                xyz1
                - target_xyz
            )
        )

        print(
            "HOLD_CHECK_1_REAL_XYZ="
            f"{fmt_xyz(xyz1)}"
        )

        print(
            "HOLD_CHECK_1_TARGET_ERROR_CM="
            f"{error1 * 100:.3f}"
        )

        # 继续保持3.5秒。
        # 如果以前的“突然回原位”来自命令断流，
        # 这一阶段应该不再发生。
        pump.hold(
            TARGET_HOLD_2_SECONDS
        )

        (
            arm2,
            head2,
            _,
            xyz2,
        ) = measure_robot(
            node,
            solver,
        )

        error2 = float(
            np.linalg.norm(
                xyz2
                - target_xyz
            )
        )

        drift = float(
            np.linalg.norm(
                xyz2
                - xyz1
            )
        )

        print(
            "HOLD_CHECK_2_REAL_XYZ="
            f"{fmt_xyz(xyz2)}"
        )

        print(
            "HOLD_CHECK_2_TARGET_ERROR_CM="
            f"{error2 * 100:.3f}"
        )

        print(
            "TARGET_HOLD_DRIFT_CM="
            f"{drift * 100:.3f}"
        )

        # ==================================
        # 8. 返回
        # ==================================

        print(
            "RETURN_START=true"
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
            "RETURN_REAL_XYZ="
            f"{fmt_xyz(final_xyz)}"
        )

        print(
            "RETURN_XYZ_ERROR_CM="
            f"{return_error * 100:.3f}"
        )

        # ==================================
        # 9. 先退出URS，再停止pump
        # ==================================

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
            > MAX_ALLOWED_PUBLISH_GAP_SEC
        ):
            raise RuntimeError(
                "50Hz命令流出现过长中断："
                f"{pump.max_gap_sec * 1000:.1f}ms"
            )

        print(
            "COMMAND_STREAM_CONTINUOUS=true"
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
            and initial_arm is not None
            and initial_head is not None
            and hand_pos is not None
        ):
            print(
                "RECOVERY=持续命令流下返回初始姿态"
            )

            try:
                if (
                    pump is not None
                ):
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
