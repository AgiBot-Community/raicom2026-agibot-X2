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

HAL_ARM_TOPIC = "/aima/hal/joint/arm/command"
HAND_STATE_TOPIC = "/aima/hal/joint/hand/state"

# -------------------------------------------------
# Cartesian规划参数
# -------------------------------------------------

# 每个XYZ waypoint最多约1 cm
WAYPOINT_STEP_M = 0.010

# 第一次运动速度：
# 0.0125 m/s -> 8 cm大约6.4秒
CARTESIAN_SPEED_MPS = 0.0125

# 修正阶段稍慢
CORRECTION_SPEED_MPS = 0.010

MIN_SEGMENT_SECONDS = 0.35

# 第一次测试暂时禁止一次移动超过10 cm
MAX_TOTAL_DISTANCE_M = 2.00

# -------------------------------------------------
# IK / 安全阈值
# -------------------------------------------------

MAX_IK_ERROR_M = 0.010

# 从起点到目标，任一右臂关节总变化
MAX_GLOBAL_JOINT_DELTA_DEG = 120.0

# 相邻Cartesian waypoint之间的关节变化
MAX_SEGMENT_JOINT_DELTA_DEG = 25.0

# 自动修正阶段最大关节变化
MAX_CORRECTION_JOINT_DELTA_DEG = 60.0

# 当前position-only IK没有锁定RPY
MAX_PATH_RPY_DELTA_DEG = 90.0

MIN_JOINT_LIMIT_MARGIN_DEG = 0.0

# 如果起始姿态本来就小于5度，不应该因此完全禁止运动。
# 但任何轨迹点都不能进入1度以内的硬危险区。
HARD_MIN_LIMIT_MARGIN_DEG = -2.0

# 真机起点若因零位/模型偏差已轻微越过名义限位，
# 最多只允许到 -1.0deg，并且只能沿“逃离限位”的方向运动。
ESCAPE_START_MIN_MARGIN_DEG = -3.0

# 对于起始时已经靠近限位的关节，
# 后续waypoint只能保持或逐渐远离限位。
# 给测量/数值误差留0.30度容差。
LIMIT_ESCAPE_TOLERANCE_DEG = 1.00

MAX_CURRENT_ARM_VELOCITY = 0.25

# -------------------------------------------------
# URS
# -------------------------------------------------

PRE_URS_HOLD_SECONDS = 2.0
POST_URS_HOLD_SECONDS = 2.0

TARGET_HOLD_SECONDS = 2.5
RETURN_HOLD_SECONDS = 1.5

MAX_TAKEOVER_HAL_ERROR_RAD = 0.08

# -------------------------------------------------
# 闭环
# -------------------------------------------------

DEFAULT_TARGET_TOLERANCE_M = 0.030
DEFAULT_MAX_CORRECTIONS = 3

# 如果第一次实际误差已经超过5cm，
# 不再自动追目标，直接安全返回
MAX_AUTO_CORRECTION_ERROR_M = 0.20

MAX_RETURN_ERROR_M = 0.080


def fmt_xyz(v):
    return (
        "["
        + ", ".join(
            f"{float(x):+.6f}"
            for x in v
        )
        + "]"
    )


def wrap_angle(rad):
    return math.atan2(
        math.sin(rad),
        math.cos(rad),
    )


def extract_max_velocity(container):
    values = []

    if isinstance(container, dict):
        for _, value in container.items():

            if isinstance(value, dict):
                if "velocity" in value:
                    values.append(
                        float(value["velocity"])
                    )

            elif hasattr(value, "velocity"):
                values.append(
                    float(value.velocity)
                )

            else:
                try:
                    values.append(
                        float(value)
                    )
                except (TypeError, ValueError):
                    pass

    else:
        arr = np.asarray(
            container,
            dtype=float,
        ).reshape(-1)

        values.extend(
            float(x)
            for x in arr
        )

    if not values:
        raise RuntimeError(
            "没有从read_state第三返回值中解析到速度"
        )

    return max(
        abs(v)
        for v in values
    )


class XYZClosedLoopNode(
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
            self._on_hal_full,
            qos,
        )

        self.create_subscription(
            HandStateArray,
            HAND_STATE_TOPIC,
            self._on_hand_state,
            qos,
        )

    def _on_hal_full(self, msg):
        self.latest_hal_arm = {
            joint.name: float(joint.position)
            for joint in msg.joints
        }

    def _on_hand_state(self, msg):
        self.latest_hand_state = msg

    def wait_for_claw_state(
        self,
        timeout_sec=5.0,
    ):
        deadline = (
            time.monotonic()
            + timeout_sec
        )

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
                "没有收到/aima/hal/joint/hand/state"
            )

        msg = self.latest_hand_state

        if int(msg.left_hand_type.value) != 2:
            raise RuntimeError(
                "当前左手不是CLAW"
            )

        if int(msg.right_hand_type.value) != 2:
            raise RuntimeError(
                "当前右手不是CLAW"
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
        deadline = (
            time.monotonic()
            + 2.0
        )

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

        missing = [
            name
            for name in ARM_POS_ORDER
            if name not in self.latest_hal_arm
        ]

        if missing:
            raise RuntimeError(
                "URS后HAL缺少关节："
                + ",".join(missing)
            )

        errors = []

        for name, target in zip(
            ARM_POS_ORDER,
            current_arm,
        ):
            errors.append(
                abs(
                    self.latest_hal_arm[name]
                    - float(target)
                )
            )

        max_index = int(
            np.argmax(errors)
        )

        max_error = float(
            errors[max_index]
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
                "URS接管后HAL目标与真实姿态偏差过大"
            )

        print(
            "TAKEOVER_VALIDATED=true"
        )


def measure_robot(
    node,
    solver,
):
    arm, head, velocity_container = (
        node.read_state()
    )

    arm = np.asarray(
        arm,
        dtype=float,
    )

    head = np.asarray(
        head,
        dtype=float,
    )

    xyz = np.asarray(
        solver.fk_xyz(
            SIDE,
            arm,
            current_head_pos=head,
        ),
        dtype=float,
    )

    return (
        arm,
        head,
        velocity_container,
        xyz,
    )


def joint_limit_margins_deg(
    solver,
    arm,
):
    """返回右臂7个关节各自距离最近机械限位的余量，单位deg。"""

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


def minimum_limit_margin_deg(
    solver,
    arm,
):
    limits = (
        solver.joint_limits_for_arm_pos()
    )[7:]

    margins = []

    for q, (
        name,
        lower,
        upper,
    ) in zip(
        arm[7:],
        limits,
    ):
        margin = min(
            float(q) - float(lower),
            float(upper) - float(q),
        )

        margins.append(
            (
                name,
                math.degrees(margin),
            )
        )

    return min(
        margins,
        key=lambda item: item[1],
    )


def plan_cartesian_path(
    solver,
    start_arm,
    head,
    start_xyz,
    target_xyz,
    *,
    waypoint_step_m,
    max_global_joint_delta_deg,
    label,
):
    start_arm = np.asarray(
        start_arm,
        dtype=float,
    )

    start_xyz = np.asarray(
        start_xyz,
        dtype=float,
    )

    target_xyz = np.asarray(
        target_xyz,
        dtype=float,
    )

    vector = (
        target_xyz
        - start_xyz
    )

    distance = float(
        np.linalg.norm(vector)
    )

    if distance < 1e-6:
        return {
            "label": label,
            "distance": 0.0,
            "waypoints": [],
            "max_global_joint_deg": 0.0,
            "max_segment_joint_deg": 0.0,
            "max_rpy_deg": 0.0,
            "min_margin_deg": 999.0,
            "min_margin_joint": "NONE",
            "final_arm": start_arm.copy(),
        }

    # 减一个极小epsilon，避免例如：
    # 0.08 / 0.01 = 8.000000000000002
    # 被ceil错误变成9。
    count = max(
        1,
        int(
            math.ceil(
                distance
                / waypoint_step_m
                - 1e-9
            )
        ),
    )

    start_rpy = np.asarray(
        solver.fk_rpy(
            SIDE,
            start_arm,
            current_head_pos=head,
        ),
        dtype=float,
    )

    previous_arm = (
        start_arm.copy()
    )

    # 起点可能本身已经靠近机械限位。
    # 这种情况下允许轨迹向远离限位的方向运动。
    start_margin_map = (
        joint_limit_margins_deg(
            solver,
            start_arm,
        )
    )

    previous_margin_map = dict(
        start_margin_map
    )

    start_min_joint = min(
        start_margin_map,
        key=start_margin_map.get,
    )

    start_min_margin_deg = (
        start_margin_map[
            start_min_joint
        ]
    )

    print(
        f"{label}_START_MIN_LIMIT_MARGIN_DEG="
        f"{start_min_margin_deg:.3f}"
    )

    print(
        f"{label}_START_MIN_LIMIT_MARGIN_JOINT="
        f"{start_min_joint}"
    )

    # 起点若只是轻微越过名义限位，允许进入“限位逃离模式”；
    # 但低于 -1deg 仍直接拒绝。
    if (
        start_min_margin_deg
        < ESCAPE_START_MIN_MARGIN_DEG
    ):
        raise RuntimeError(
            f"{label} 起始姿态超过允许的限位逃离范围："
            f"{start_min_joint}, "
            f"{start_min_margin_deg:.3f}deg"
        )

    if (
        start_min_margin_deg
        < HARD_MIN_LIMIT_MARGIN_DEG
    ):
        print(
            f"{label}_LIMIT_ESCAPE_MODE=true"
        )
        print(
            f"{label}_LIMIT_ESCAPE_START_MARGIN_DEG="
            f"{start_min_margin_deg:.3f}"
        )

    waypoints = []

    max_global = 0.0
    max_segment = 0.0
    max_rpy = 0.0

    min_margin_deg = math.inf
    min_margin_joint = ""

    segment_xyz_distance = (
        distance / count
    )

    for i in range(
        1,
        count + 1,
    ):
        alpha = (
            i / count
        )

        waypoint_xyz = (
            start_xyz
            + vector * alpha
        )

        result = (
            solver.solve_position(
                SIDE,
                waypoint_xyz,
                previous_arm,
                current_head_pos=head,
            )
        )

        if not result.success:
            raise RuntimeError(
                f"{label} waypoint {i}/{count} "
                "IK未收敛"
            )

        q = np.asarray(
            result.arm_pos,
            dtype=float,
        )

        final_xyz = np.asarray(
            result.final_xyz,
            dtype=float,
        )

        ik_error = float(
            np.linalg.norm(
                final_xyz
                - waypoint_xyz
            )
        )

        if ik_error > MAX_IK_ERROR_M:
            raise RuntimeError(
                f"{label} waypoint {i}/{count} "
                f"IK误差={ik_error:.6f}m"
            )

        # 右臂IK只允许控制右臂。
        # solver返回14关节结果时可能带来少量左臂数值漂移，
        # 不把这种数值漂移当成规划失败，而是直接固定左臂。
        left_change = float(
            np.max(
                np.abs(
                    np.degrees(
                        q[:7]
                        - start_arm[:7]
                    )
                )
            )
        )

        if i == 1 and left_change > 0.01:
            print(
                f"{label}_SOLVER_LEFT_DRIFT_DEG="
                f"{left_change:.3f}"
            )
            print(
                f"{label}_LEFT_ARM_PINNED=true"
            )

        # 强制保持左臂真实起始姿态
        q[:7] = start_arm[:7]

        segment_delta = np.degrees(
            q[7:]
            - previous_arm[7:]
        )

        segment_max = float(
            np.max(
                np.abs(segment_delta)
            )
        )

        max_segment = max(
            max_segment,
            segment_max,
        )

        if (
            segment_max
            > MAX_SEGMENT_JOINT_DELTA_DEG
        ):
            raise RuntimeError(
                f"{label} waypoint {i}/{count} "
                "相邻IK解跳变过大："
                f"{segment_max:.3f}deg"
            )

        global_delta = np.degrees(
            q[7:]
            - start_arm[7:]
        )

        global_max = float(
            np.max(
                np.abs(global_delta)
            )
        )

        max_global = max(
            max_global,
            global_max,
        )

        if (
            global_max
            > max_global_joint_delta_deg
        ):
            raise RuntimeError(
                f"{label}总关节变化过大："
                f"{global_max:.3f}deg"
            )

        current_margin_map = (
            joint_limit_margins_deg(
                solver,
                q,
            )
        )

        margin_joint = min(
            current_margin_map,
            key=current_margin_map.get,
        )

        margin_deg = (
            current_margin_map[
                margin_joint
            ]
        )

        if margin_deg < min_margin_deg:
            min_margin_deg = (
                margin_deg
            )
            min_margin_joint = (
                margin_joint
            )

        # --------------------------------------------
        # 限位规则：
        #
        # 1. 无论如何都不能进入1度以内硬危险区。
        #
        # 2. 如果起点本身距离限位>=5度，
        #    整条路径仍要求>=5度。
        #
        # 3. 如果起点本来已经<5度，
        #    允许向外逃离，但不能继续明显靠近限位。
        # --------------------------------------------

        for (
            joint_name,
            current_margin,
        ) in current_margin_map.items():

            start_margin = (
                start_margin_map[
                    joint_name
                ]
            )

            previous_margin = (
                previous_margin_map[
                    joint_name
                ]
            )

            # 正常情况下不能进入1deg以内硬危险区。
            # 唯一例外：起点本来就在1deg以内，
            # 且上一点也尚未逃出1deg，
            # 则允许在不低于-1deg范围内继续向安全方向逃离。
            if (
                current_margin
                < HARD_MIN_LIMIT_MARGIN_DEG
            ):
                if (
                    start_margin
                    >= HARD_MIN_LIMIT_MARGIN_DEG
                    or previous_margin
                    >= HARD_MIN_LIMIT_MARGIN_DEG
                    or current_margin
                    < ESCAPE_START_MIN_MARGIN_DEG
                ):
                    raise RuntimeError(
                        f"{label} waypoint {i}/{count} "
                        "进入关节限位硬危险区："
                        f"{joint_name}, "
                        f"{current_margin:.3f}deg"
                    )

            # 起点本来安全，则全过程必须保持5度以上
            if (
                start_margin
                >= MIN_JOINT_LIMIT_MARGIN_DEG
            ):
                if (
                    current_margin
                    < MIN_JOINT_LIMIT_MARGIN_DEG
                ):
                    raise RuntimeError(
                        f"{label} waypoint {i}/{count} "
                        "从安全区进入限位缓冲区："
                        f"{joint_name}, "
                        f"{current_margin:.3f}deg"
                    )

            # 起点本来就在5度以内：
            # 允许向外走，但禁止进一步明显靠近。
            else:
                if (
                    current_margin
                    + LIMIT_ESCAPE_TOLERANCE_DEG
                    < previous_margin
                ):
                    raise RuntimeError(
                        f"{label} waypoint {i}/{count} "
                        "起点已靠近限位且轨迹继续向限位靠近："
                        f"{joint_name}, "
                        f"previous={previous_margin:.3f}deg, "
                        f"current={current_margin:.3f}deg"
                    )

        previous_margin_map = dict(
            current_margin_map
        )

        rpy = np.asarray(
            solver.fk_rpy(
                SIDE,
                q,
                current_head_pos=head,
            ),
            dtype=float,
        )

        delta_rpy = np.asarray(
            [
                wrap_angle(
                    rpy[j]
                    - start_rpy[j]
                )
                for j in range(3)
            ]
        )

        rpy_deg = float(
            np.max(
                np.abs(
                    np.degrees(
                        delta_rpy
                    )
                )
            )
        )

        max_rpy = max(
            max_rpy,
            rpy_deg,
        )

        if (
            rpy_deg
            > MAX_PATH_RPY_DELTA_DEG
        ):
            print(
                f"{label}_RPY_WARNING="
                f"waypoint {i}/{count}, "
                f"{rpy_deg:.3f}deg"
            )

        waypoints.append(
            {
                "index": i,
                "count": count,
                "xyz": waypoint_xyz,
                "arm": q,
                "ik_error": ik_error,
                "xyz_step": (
                    segment_xyz_distance
                ),
            }
        )

        previous_arm = q

    return {
        "label": label,
        "distance": distance,
        "waypoints": waypoints,
        "max_global_joint_deg": max_global,
        "max_segment_joint_deg": max_segment,
        "max_rpy_deg": max_rpy,
        "min_margin_deg": min_margin_deg,
        "min_margin_joint": min_margin_joint,
        "final_arm": (
            previous_arm.copy()
        ),
    }


def print_plan(
    plan,
):
    print(
        f"{plan['label']}_DISTANCE_M="
        f"{plan['distance']:.6f}"
    )

    print(
        f"{plan['label']}_DISTANCE_CM="
        f"{plan['distance'] * 100:.3f}"
    )

    print(
        f"{plan['label']}_WAYPOINTS="
        f"{len(plan['waypoints'])}"
    )

    print(
        f"{plan['label']}_MAX_GLOBAL_JOINT_DEG="
        f"{plan['max_global_joint_deg']:.3f}"
    )

    print(
        f"{plan['label']}_MAX_SEGMENT_JOINT_DEG="
        f"{plan['max_segment_joint_deg']:.3f}"
    )

    print(
        f"{plan['label']}_MAX_RPY_DEG="
        f"{plan['max_rpy_deg']:.3f}"
    )

    print(
        f"{plan['label']}_MIN_LIMIT_MARGIN_DEG="
        f"{plan['min_margin_deg']:.3f}"
    )

    print(
        f"{plan['label']}_MIN_LIMIT_MARGIN_JOINT="
        f"{plan['min_margin_joint']}"
    )


def execute_plan(
    node,
    plan,
    start_arm,
    head,
    hand,
    *,
    speed_mps,
):
    previous_arm = np.asarray(
        start_arm,
        dtype=float,
    )

    total_frames = 0
    total_seconds = 0.0

    for wp in plan["waypoints"]:

        duration = max(
            MIN_SEGMENT_SECONDS,
            float(wp["xyz_step"])
            / speed_mps,
        )

        frames = (
            node.publish_segment(
                previous_arm,
                wp["arm"],
                head,
                hand,
                duration,
            )
        )

        total_frames += frames
        total_seconds += duration

        previous_arm = (
            wp["arm"]
        )

    return (
        total_frames,
        total_seconds,
        previous_arm,
    )


def main():

    parser = argparse.ArgumentParser(
        description=(
            "比赛版X2右臂闭环XYZ测试"
        )
    )

    target_group = (
        parser.add_mutually_exclusive_group(
            required=True
        )
    )

    target_group.add_argument(
        "--offset",
        nargs=3,
        type=float,
        metavar=(
            "DX",
            "DY",
            "DZ",
        ),
        help=(
            "相对当前末端XYZ偏移，单位m"
        ),
    )

    target_group.add_argument(
        "--target",
        nargs=3,
        type=float,
        metavar=(
            "X",
            "Y",
            "Z",
        ),
        help=(
            "绝对目标XYZ，单位m"
        ),
    )

    parser.add_argument(
        "--execute",
        action="store_true",
        help="允许真实机械臂运动",
    )

    parser.add_argument(
        "--tolerance",
        type=float,
        default=(
            DEFAULT_TARGET_TOLERANCE_M
        ),
        help=(
            "最终XYZ容差，默认0.01m"
        ),
    )

    parser.add_argument(
        "--max-corrections",
        type=int,
        default=(
            DEFAULT_MAX_CORRECTIONS
        ),
        help=(
            "最多终点自动修正次数，默认2"
        ),
    )

    args = parser.parse_args()

    if args.tolerance <= 0:
        raise SystemExit(
            "--tolerance必须>0"
        )

    if (
        args.max_corrections < 0
        or args.max_corrections > 5
    ):
        raise SystemExit(
            "--max-corrections范围0~5"
        )

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
        # =========================================
        # 当前真实状态
        # =========================================

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

        total_distance = float(
            np.linalg.norm(offset)
        )

        if (
            total_distance
            > MAX_TOTAL_DISTANCE_M
        ):
            raise RuntimeError(
                "第一次闭环测试禁止一次移动超过"
                f"{MAX_TOTAL_DISTANCE_M * 100:.1f}cm，"
                f"当前={total_distance * 100:.2f}cm"
            )

        # =========================================
        # 初始Cartesian规划
        # =========================================

        initial_plan = (
            plan_cartesian_path(
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
        )

        print(
            "=== 比赛版X2右臂闭环XYZ测试 ==="
        )

        print(
            f"EXECUTE={str(args.execute).lower()}"
        )

        print(
            "SIDE=RIGHT"
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
            "TARGET_TOLERANCE_M="
            f"{args.tolerance:.6f}"
        )

        print(
            "MAX_CORRECTIONS="
            f"{args.max_corrections}"
        )

        print(
            "CARTESIAN_WAYPOINT_STEP_M="
            f"{WAYPOINT_STEP_M:.6f}"
        )

        print(
            "INPUT_SOURCE_SERVICE_USED=false"
        )

        print(
            "HAND_MODE=CLAW_OPEN_CLOSE"
        )

        print(
            "HAND_POS="
            + ",".join(
                f"{x:.6f}"
                for x in hand_pos
            )
        )

        print_plan(
            initial_plan
        )

        if not args.execute:
            print(
                "REAL_ROBOT_MOTION=false"
            )

            print(
                "RESULT=PASS"
            )

            return 0

        # =========================================
        # 真机前检查ROS图
        # =========================================

        subscribers = (
            node.wait_for_command_subscriber(
                timeout_sec=5.0
            )
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command没有订阅者"
            )

        publishers = (
            node.count_publishers(
                COMMAND_TOPIC
            )
        )

        # 当前脚本自己的publisher计数为1
        if publishers > 1:
            raise RuntimeError(
                "检测到其他上肢command publisher："
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

        # =========================================
        # URS接管
        # =========================================

        frames = node.publish_segment(
            initial_arm,
            initial_arm,
            initial_head,
            hand_pos,
            PRE_URS_HOLD_SECONDS,
        )

        print(
            f"PRE_URS_FRAMES={frames}"
        )

        node.latest_hal_arm.clear()

        node.set_mode_while_holding(
            "UPPERBODY_REMOTE_SPLIT",
            initial_arm,
            initial_head,
            hand_pos,
        )

        switched_to_urs = True

        frames = node.publish_segment(
            initial_arm,
            initial_arm,
            initial_head,
            hand_pos,
            POST_URS_HOLD_SECONDS,
        )

        print(
            f"POST_URS_HOLD_FRAMES={frames}"
        )

        node.validate_hal_takeover(
            initial_arm
        )

        # =========================================
        # 第一轮Cartesian运动
        # =========================================

        print(
            "INITIAL_MOTION_START=true"
        )

        (
            frames,
            seconds,
            commanded_final_arm,
        ) = execute_plan(
            node,
            initial_plan,
            initial_arm,
            initial_head,
            hand_pos,
            speed_mps=(
                CARTESIAN_SPEED_MPS
            ),
        )

        print(
            f"INITIAL_MOTION_FRAMES={frames}"
        )

        print(
            "INITIAL_MOTION_SECONDS="
            f"{seconds:.3f}"
        )

        frames = node.publish_segment(
            commanded_final_arm,
            commanded_final_arm,
            initial_head,
            hand_pos,
            TARGET_HOLD_SECONDS,
        )

        print(
            f"INITIAL_TARGET_HOLD_FRAMES={frames}"
        )

        # =========================================
        # 终点测量 + 自动修正
        # =========================================

        corrections_used = 0

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

        while (
            target_error
            > args.tolerance
            and corrections_used
            < args.max_corrections
        ):

            if (
                target_error
                > MAX_AUTO_CORRECTION_ERROR_M
            ):
                print(
                    "AUTO_CORRECTION_ABORT=true"
                )

                print(
                    "AUTO_CORRECTION_ABORT_REASON="
                    "当前误差超过5cm"
                )

                break

            corrections_used += 1

            print(
                "CORRECTION_START="
                f"{corrections_used}"
            )

            correction_plan = (
                plan_cartesian_path(
                    solver,
                    actual_arm,
                    actual_head,
                    actual_xyz,
                    target_xyz,
                    waypoint_step_m=0.008,
                    max_global_joint_delta_deg=(
                        MAX_CORRECTION_JOINT_DELTA_DEG
                    ),
                    label=(
                        f"CORRECTION_{corrections_used}"
                    ),
                )
            )

            print_plan(
                correction_plan
            )

            (
                frames,
                seconds,
                correction_final_arm,
            ) = execute_plan(
                node,
                correction_plan,
                actual_arm,
                initial_head,
                hand_pos,
                speed_mps=(
                    CORRECTION_SPEED_MPS
                ),
            )

            print(
                "CORRECTION_FRAMES="
                f"{frames}"
            )

            print(
                "CORRECTION_SECONDS="
                f"{seconds:.3f}"
            )

            frames = node.publish_segment(
                correction_final_arm,
                correction_final_arm,
                initial_head,
                hand_pos,
                TARGET_HOLD_SECONDS,
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

            print(
                f"PASS_{corrections_used}_REAL_XYZ="
                f"{fmt_xyz(actual_xyz)}"
            )

            print(
                f"PASS_{corrections_used}_TARGET_ERROR_M="
                f"{target_error:.6f}"
            )

            print(
                f"PASS_{corrections_used}_TARGET_ERROR_CM="
                f"{target_error * 100:.3f}"
            )

        target_reached = (
            target_error
            <= args.tolerance
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

        # =========================================
        # 无论终点是否达标，都返回起点
        # =========================================

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

        return_plan = (
            plan_cartesian_path(
                solver,
                return_start_arm,
                return_start_head,
                return_start_xyz,
                start_xyz,
                waypoint_step_m=(
                    WAYPOINT_STEP_M
                ),
                max_global_joint_delta_deg=120.0,
                label="RETURN_PLAN",
            )
        )

        print_plan(
            return_plan
        )

        (
            frames,
            seconds,
            return_command_arm,
        ) = execute_plan(
            node,
            return_plan,
            return_start_arm,
            initial_head,
            hand_pos,
            speed_mps=(
                CARTESIAN_SPEED_MPS
            ),
        )

        print(
            f"RETURN_FRAMES={frames}"
        )

        print(
            "RETURN_SECONDS="
            f"{seconds:.3f}"
        )

        # 最后明确保持最初真实关节姿态
        frames = node.publish_segment(
            return_command_arm,
            initial_arm,
            initial_head,
            hand_pos,
            RETURN_HOLD_SECONDS,
        )

        print(
            f"RETURN_HOLD_FRAMES={frames}"
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

        # =========================================
        # 回稳定站立
        # =========================================

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
                "REASON=自动修正后仍未达到目标XYZ容差"
            )

            return 1

        if (
            return_error
            > MAX_RETURN_ERROR_M
        ):
            print(
                "RETURN_ERROR_WARNING=true"
            )

            print(
                "RETURN_ERROR_WARNING_CM="
                f"{return_error * 100:.3f}"
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
                "RECOVERY=尝试从当前真实状态缓慢返回初始关节姿态"
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
