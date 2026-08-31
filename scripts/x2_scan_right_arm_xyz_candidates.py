#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
    SafeJointMicrotest,
)


SIDE = ArmSide.RIGHT

# 扫描 5 cm 和 8 cm，6个基本方向
DISTANCES = [0.05, 0.08]

DIRECTIONS = [
    ("X+", np.array([+1.0,  0.0,  0.0])),
    ("X-", np.array([-1.0,  0.0,  0.0])),
    ("Y+", np.array([ 0.0, +1.0,  0.0])),
    ("Y-", np.array([ 0.0, -1.0,  0.0])),
    ("Z+", np.array([ 0.0,  0.0, +1.0])),
    ("Z-", np.array([ 0.0,  0.0, -1.0])),
]

# 第一次真机XYZ运动的建议门槛
A_MAX_JOINT_DEG = 20.0
A_MAX_RPY_DEG = 30.0
A_MIN_LIMIT_MARGIN_RAD = 0.10
A_MAX_ERROR_M = 0.002

B_MAX_JOINT_DEG = 25.0
B_MAX_RPY_DEG = 45.0
B_MIN_LIMIT_MARGIN_RAD = 0.05
B_MAX_ERROR_M = 0.003


def wrap_angle(rad):
    return math.atan2(
        math.sin(rad),
        math.cos(rad),
    )


def fmt_xyz(xyz):
    return "[" + ", ".join(
        f"{float(v):+.6f}"
        for v in xyz
    ) + "]"


def fmt_deg(values):
    return "[" + ", ".join(
        f"{math.degrees(float(v)):+.2f}"
        for v in values
    ) + "]"


def classify(
    success,
    final_error,
    max_joint_deg,
    max_rpy_deg,
    limit_margin,
):
    if not success:
        return "FAIL"

    if (
        final_error <= A_MAX_ERROR_M
        and max_joint_deg <= A_MAX_JOINT_DEG
        and max_rpy_deg <= A_MAX_RPY_DEG
        and limit_margin >= A_MIN_LIMIT_MARGIN_RAD
    ):
        return "A"

    if (
        final_error <= B_MAX_ERROR_M
        and max_joint_deg <= B_MAX_JOINT_DEG
        and max_rpy_deg <= B_MAX_RPY_DEG
        and limit_margin >= B_MIN_LIMIT_MARGIN_RAD
    ):
        return "B"

    return "C"


def main():
    rclpy.init()

    node = SafeJointMicrotest()

    try:
        config = X2IKConfig.default_omnipicker()
        solver = X2ArmIKSolver(config)

        current_arm, current_head, velocities = (
            node.read_state()
        )

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
                f"真实arm_pos长度不是14: {current_arm.shape}"
            )

        current_xyz = np.asarray(
            solver.fk_xyz(
                SIDE,
                current_arm,
                current_head_pos=current_head,
            ),
            dtype=float,
        )

        current_rpy = np.asarray(
            solver.fk_rpy(
                SIDE,
                current_arm,
                current_head_pos=current_head,
            ),
            dtype=float,
        )

        limits = (
            solver.joint_limits_for_arm_pos()
        )

        if len(limits) != 14:
            raise RuntimeError(
                f"关节限位数量不是14: {len(limits)}"
            )

        right_limits = limits[7:]

        print(
            "=== X2右臂XYZ候选点离线扫描 ==="
        )
        print(
            "REAL_ROBOT_MOTION=false"
        )
        print(
            "SIDE=RIGHT"
        )
        print(
            f"CURRENT_XYZ={fmt_xyz(current_xyz)}"
        )
        print(
            f"CURRENT_RPY_RAD={fmt_xyz(current_rpy)}"
        )
        print(
            f"CURRENT_RPY_DEG={fmt_deg(current_rpy)}"
        )
        print(
            "SCAN_DISTANCES_M=0.05,0.08"
        )
        print()

        candidates = []

        candidate_id = 0

        for distance in DISTANCES:
            for direction_name, direction in DIRECTIONS:
                candidate_id += 1

                target_xyz = (
                    current_xyz
                    + distance * direction
                )

                result = solver.solve_position(
                    SIDE,
                    target_xyz,
                    current_arm,
                    current_head_pos=current_head,
                )

                result_arm = np.asarray(
                    result.arm_pos,
                    dtype=float,
                )

                final_xyz = np.asarray(
                    result.final_xyz,
                    dtype=float,
                )

                final_error = float(
                    np.linalg.norm(
                        target_xyz - final_xyz
                    )
                )

                # 右臂7关节变化
                right_delta = (
                    result_arm[7:]
                    - current_arm[7:]
                )

                right_delta_deg = np.degrees(
                    right_delta
                )

                max_joint_deg = float(
                    np.max(
                        np.abs(right_delta_deg)
                    )
                )

                max_joint_idx = int(
                    np.argmax(
                        np.abs(right_delta_deg)
                    )
                )

                max_joint_name = (
                    right_limits[
                        max_joint_idx
                    ][0]
                )

                # 左臂理论上不应该变化
                left_delta_deg = np.degrees(
                    result_arm[:7]
                    - current_arm[:7]
                )

                passive_max_deg = float(
                    np.max(
                        np.abs(left_delta_deg)
                    )
                )

                # IK后末端姿态变化
                final_rpy = np.asarray(
                    solver.fk_rpy(
                        SIDE,
                        result_arm,
                        current_head_pos=current_head,
                    ),
                    dtype=float,
                )

                rpy_delta = np.asarray(
                    [
                        wrap_angle(
                            final_rpy[i]
                            - current_rpy[i]
                        )
                        for i in range(3)
                    ],
                    dtype=float,
                )

                rpy_delta_deg = np.degrees(
                    rpy_delta
                )

                max_rpy_deg = float(
                    np.max(
                        np.abs(rpy_delta_deg)
                    )
                )

                # 目标位姿距离关节上下限的最小余量
                margins = []

                for q, (
                    joint_name,
                    lower,
                    upper,
                ) in zip(
                    result_arm[7:],
                    right_limits,
                ):
                    margins.append(
                        min(
                            float(q) - lower,
                            upper - float(q),
                        )
                    )

                min_limit_margin = float(
                    min(margins)
                )

                rating = classify(
                    result.success,
                    final_error,
                    max_joint_deg,
                    max_rpy_deg,
                    min_limit_margin,
                )

                # 越低越保守。
                risk_score = (
                    max_joint_deg
                    + 0.40 * max_rpy_deg
                    + 1000.0 * final_error
                )

                candidate = {
                    "id": candidate_id,
                    "distance": distance,
                    "direction": direction_name,
                    "target_xyz": target_xyz,
                    "final_xyz": final_xyz,
                    "success": bool(result.success),
                    "iterations": int(result.iterations),
                    "error": final_error,
                    "right_delta_deg": right_delta_deg,
                    "passive_max_deg": passive_max_deg,
                    "max_joint_deg": max_joint_deg,
                    "max_joint_name": max_joint_name,
                    "rpy_delta_deg": rpy_delta_deg,
                    "max_rpy_deg": max_rpy_deg,
                    "limit_margin": min_limit_margin,
                    "rating": rating,
                    "risk_score": risk_score,
                    "arm_pos": result_arm,
                }

                candidates.append(candidate)

                print(
                    "========================================"
                )
                print(
                    f"CANDIDATE={candidate_id}"
                )
                print(
                    f"DIRECTION={direction_name}"
                )
                print(
                    f"DISTANCE_CM={distance * 100:.1f}"
                )
                print(
                    f"TARGET_XYZ={fmt_xyz(target_xyz)}"
                )
                print(
                    f"IK_SUCCESS={result.success}"
                )
                print(
                    f"IK_MESSAGE={result.message}"
                )
                print(
                    f"ITERATIONS={result.iterations}"
                )
                print(
                    f"FINAL_XYZ={fmt_xyz(final_xyz)}"
                )
                print(
                    f"FINAL_ERROR_M={final_error:.9f}"
                )
                print(
                    "RIGHT_DELTA_DEG="
                    + "["
                    + ", ".join(
                        f"{v:+.2f}"
                        for v in right_delta_deg
                    )
                    + "]"
                )
                print(
                    "RIGHT_JOINT_ORDER="
                    + "["
                    + ", ".join(
                        item[0]
                        for item in right_limits
                    )
                    + "]"
                )
                print(
                    f"MAX_JOINT_DELTA_DEG={max_joint_deg:.3f}"
                )
                print(
                    f"MAX_JOINT={max_joint_name}"
                )
                print(
                    f"LEFT_ARM_MAX_DELTA_DEG={passive_max_deg:.6f}"
                )
                print(
                    "RPY_DELTA_DEG="
                    + "["
                    + ", ".join(
                        f"{v:+.2f}"
                        for v in rpy_delta_deg
                    )
                    + "]"
                )
                print(
                    f"MAX_RPY_DELTA_DEG={max_rpy_deg:.3f}"
                )
                print(
                    "MIN_JOINT_LIMIT_MARGIN_RAD="
                    f"{min_limit_margin:.6f}"
                )
                print(
                    "MIN_JOINT_LIMIT_MARGIN_DEG="
                    f"{math.degrees(min_limit_margin):.3f}"
                )
                print(
                    f"RATING={rating}"
                )
                print(
                    f"RISK_SCORE={risk_score:.3f}"
                )
                print()

        print(
            "========== 排名汇总 =========="
        )

        rating_order = {
            "A": 0,
            "B": 1,
            "C": 2,
            "FAIL": 3,
        }

        ranked = sorted(
            candidates,
            key=lambda x: (
                rating_order[x["rating"]],
                x["risk_score"],
            ),
        )

        for rank, c in enumerate(
            ranked,
            start=1,
        ):
            print(
                f"RANK={rank} "
                f"CANDIDATE={c['id']} "
                f"{c['direction']} "
                f"{c['distance'] * 100:.0f}cm "
                f"RATING={c['rating']} "
                f"MAX_JOINT={c['max_joint_deg']:.2f}deg "
                f"MAX_RPY={c['max_rpy_deg']:.2f}deg "
                f"ERR={c['error']:.6f}m"
            )

        # 优先找8cm的A类。
        recommended = None

        safe_a_8 = [
            c for c in candidates
            if (
                c["rating"] == "A"
                and abs(c["distance"] - 0.08)
                < 1e-9
            )
        ]

        if safe_a_8:
            recommended = min(
                safe_a_8,
                key=lambda x: x["risk_score"],
            )
        else:
            safe_a = [
                c for c in candidates
                if c["rating"] == "A"
            ]

            if safe_a:
                recommended = min(
                    safe_a,
                    key=lambda x: x["risk_score"],
                )
            else:
                safe_b = [
                    c for c in candidates
                    if c["rating"] == "B"
                ]

                if safe_b:
                    recommended = min(
                        safe_b,
                        key=lambda x: x["risk_score"],
                    )

        print()
        print(
            "========== 自动推荐 =========="
        )

        if recommended is None:
            print(
                "RECOMMENDED=NONE"
            )
            print(
                "REASON=没有A/B级候选，不建议直接进行XYZ真机运动"
            )
        else:
            print(
                f"RECOMMENDED_CANDIDATE={recommended['id']}"
            )
            print(
                f"RECOMMENDED_DIRECTION={recommended['direction']}"
            )
            print(
                "RECOMMENDED_DISTANCE_CM="
                f"{recommended['distance'] * 100:.1f}"
            )
            print(
                "RECOMMENDED_TARGET_XYZ="
                f"{fmt_xyz(recommended['target_xyz'])}"
            )
            print(
                "RECOMMENDED_RATING="
                f"{recommended['rating']}"
            )
            print(
                "RECOMMENDED_MAX_JOINT_DELTA_DEG="
                f"{recommended['max_joint_deg']:.3f}"
            )
            print(
                "RECOMMENDED_MAX_RPY_DELTA_DEG="
                f"{recommended['max_rpy_deg']:.3f}"
            )
            print(
                "RECOMMENDED_FINAL_ERROR_M="
                f"{recommended['error']:.9f}"
            )

        print()
        print(
            "REAL_ROBOT_MOTION=false"
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
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
