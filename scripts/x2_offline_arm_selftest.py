#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
X2机械臂零动作闭环自检。

执行内容：
1. 读取真机机械臂关节状态；
2. 检查14关节名称、数值、错误码和关节限位；
3. 对当前左右末端执行FK；
4. 对当前末端位置执行原位IK回算；
5. 尝试5 mm虚拟笛卡尔位移IK；
6. 在内存中使用Ruckig生成关节轨迹。

本程序不发送任何机械臂或夹爪控制命令。
"""

import math
import sys
from typing import Iterable

import rclpy
from rclpy.node import Node

from aimdk_msgs.srv import GetAllJointState
from ruckig import (
    InputParameter,
    OutputParameter,
    Result,
    Ruckig,
)

from x2_ik_sdk import X2ArmIKSolver, X2IKConfig
from x2_ik_sdk.config import ARM_POS_ORDER, ArmSide


SERVICE_NAME = "/aimdk_5Fmsgs/srv/GetAllJointState"

DOF = 14
CONTROL_DT = 0.002
SERVICE_WAIT_SEC = 5.0
SERVICE_CALL_SEC = 10.0

IK_ERROR_LIMIT = 5e-4
INACTIVE_ARM_LIMIT = 1e-6
SELF_IK_JOINT_WARN = 0.10

MAX_VELOCITY = 1.0
MAX_ACCELERATION = 1.0
MAX_JERK = 25.0

MAX_RUCKIG_STEPS = 15000


def max_abs(values: Iterable[float]) -> float:
    values = list(values)
    return max((abs(float(value)) for value in values), default=0.0)


def max_difference(
    first: Iterable[float],
    second: Iterable[float],
) -> float:
    return max_abs(
        float(a) - float(b)
        for a, b in zip(first, second)
    )


def require_finite(label: str, values: Iterable[float]) -> None:
    invalid = [
        (index, value)
        for index, value in enumerate(values)
        if not math.isfinite(float(value))
    ]

    if invalid:
        raise RuntimeError(
            f"{label} contains non-finite values: {invalid}"
        )


def read_arm_state():
    rclpy.init()

    node = Node("x2_offline_arm_selftest")
    client = node.create_client(
        GetAllJointState,
        SERVICE_NAME,
    )

    try:
        if not client.wait_for_service(
            timeout_sec=SERVICE_WAIT_SEC
        ):
            raise RuntimeError(
                f"service unavailable: {SERVICE_NAME}"
            )

        request = GetAllJointState.Request()
        request.request.header.stamp = (
            node.get_clock().now().to_msg()
        )

        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            node,
            future,
            timeout_sec=SERVICE_CALL_SEC,
        )

        if not future.done():
            raise RuntimeError(
                "GetAllJointState call timed out"
            )

        exception = future.exception()

        if exception is not None:
            raise RuntimeError(
                f"GetAllJointState exception: {exception!r}"
            )

        response = future.result()

        if response is None:
            raise RuntimeError(
                "GetAllJointState returned no response"
            )

        return response

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


def parse_arm_state(response):
    states = list(response.arm_joints)

    print("SERVICE_NAME:", SERVICE_NAME)
    print(
        "SERVICE_RESPONSE_CODE:",
        response.reponse.header.code,
    )
    print(
        "SERVICE_RESPONSE_STATUS:",
        response.reponse.status.value,
    )
    print(
        "SERVICE_RESPONSE_MESSAGE:",
        repr(response.reponse.message),
    )
    print("RAW_ARM_JOINT_COUNT:", len(states))

    if response.reponse.header.code != 0:
        raise RuntimeError(
            "GetAllJointState response code is nonzero: "
            f"{response.reponse.header.code}"
        )

    if len(states) != DOF:
        raise RuntimeError(
            f"expected {DOF} arm joints, got {len(states)}"
        )

    by_name = {}

    for state in states:
        if not state.name:
            raise RuntimeError(
                "received arm joint with empty name"
            )

        if state.name in by_name:
            raise RuntimeError(
                f"duplicate arm joint: {state.name}"
            )

        by_name[state.name] = state

    missing = [
        name
        for name in ARM_POS_ORDER
        if name not in by_name
    ]

    unexpected = sorted(
        set(by_name) - set(ARM_POS_ORDER)
    )

    if missing:
        raise RuntimeError(
            f"missing arm joints: {missing}"
        )

    if unexpected:
        raise RuntimeError(
            f"unexpected arm joints: {unexpected}"
        )

    positions = [
        float(by_name[name].position)
        for name in ARM_POS_ORDER
    ]

    velocities = [
        float(by_name[name].velocity)
        for name in ARM_POS_ORDER
    ]

    efforts = [
        float(by_name[name].effort)
        for name in ARM_POS_ORDER
    ]

    error_codes = [
        int(by_name[name].error_code)
        for name in ARM_POS_ORDER
    ]

    require_finite("positions", positions)
    require_finite("velocities", velocities)
    require_finite("efforts", efforts)

    bad_error_codes = [
        (ARM_POS_ORDER[index], code)
        for index, code in enumerate(error_codes)
        if code != 0
    ]

    if bad_error_codes:
        raise RuntimeError(
            f"joint error codes are nonzero: "
            f"{bad_error_codes}"
        )

    print("ARM_JOINT_COUNT=14")
    print("ARM_ORDER_OK=true")
    print("ARM_ERROR_CODES_OK=true")
    print(
        "MAX_ABS_CURRENT_VELOCITY:",
        f"{max_abs(velocities):.8f}",
    )

    for index, name in enumerate(ARM_POS_ORDER):
        print(
            f"ARM_{index:02d} "
            f"name={name} "
            f"position={positions[index]:.8f} "
            f"velocity={velocities[index]:.8f} "
            f"effort={efforts[index]:.8f} "
            f"error_code={error_codes[index]}"
        )

    return positions, velocities


def check_joint_limits(
    solver: X2ArmIKSolver,
    arm_pos: list[float],
):
    limits = solver.joint_limits_for_arm_pos()

    if len(limits) != DOF:
        raise RuntimeError(
            f"expected {DOF} joint limits, got {len(limits)}"
        )

    violations = []

    for index, item in enumerate(limits):
        name, lower, upper = item
        expected_name = ARM_POS_ORDER[index]
        position = arm_pos[index]

        if name != expected_name:
            raise RuntimeError(
                "joint-limit order mismatch: "
                f"index={index}, limit_name={name}, "
                f"expected={expected_name}"
            )

        if position < lower or position > upper:
            violations.append(
                (
                    name,
                    position,
                    lower,
                    upper,
                )
            )

    if violations:
        raise RuntimeError(
            f"joint-limit violations: {violations}"
        )

    print("CURRENT_JOINT_LIMIT_CHECK=PASS")

    return limits


def active_slice(side: ArmSide):
    if side == ArmSide.LEFT:
        return slice(0, 7)

    return slice(7, 14)


def inactive_slice(side: ArmSide):
    if side == ArmSide.LEFT:
        return slice(7, 14)

    return slice(0, 7)


def test_side(
    solver: X2ArmIKSolver,
    side: ArmSide,
    current_arm_pos: list[float],
):
    side_label = side.value.upper()

    current_xyz = solver.fk_xyz(
        side,
        current_arm_pos=current_arm_pos,
    )

    current_rpy = solver.fk_rpy(
        side,
        current_arm_pos=current_arm_pos,
    )

    require_finite(
        f"{side_label}_CURRENT_XYZ",
        current_xyz,
    )
    require_finite(
        f"{side_label}_CURRENT_RPY",
        current_rpy,
    )

    print(
        f"{side_label}_CURRENT_XYZ="
        f"{[round(value, 9) for value in current_xyz]}"
    )
    print(
        f"{side_label}_CURRENT_RPY="
        f"{[round(value, 9) for value in current_rpy]}"
    )

    self_result = solver.solve_position(
        side,
        target_xyz=current_xyz,
        current_arm_pos=current_arm_pos,
    )

    if not self_result.success:
        raise RuntimeError(
            f"{side_label} self IK failed: "
            f"{self_result.message}"
        )

    if self_result.error_norm > IK_ERROR_LIMIT:
        raise RuntimeError(
            f"{side_label} self IK error too large: "
            f"{self_result.error_norm}"
        )

    side_active = active_slice(side)

    self_joint_delta = max_difference(
        self_result.arm_pos[side_active],
        current_arm_pos[side_active],
    )

    print(
        f"{side_label}_SELF_IK=PASS "
        f"error_norm={self_result.error_norm:.10f} "
        f"iterations={self_result.iterations} "
        f"max_active_joint_delta={self_joint_delta:.10f}"
    )

    if self_joint_delta > SELF_IK_JOINT_WARN:
        print(
            f"{side_label}_SELF_IK_JOINT_DELTA=WARN "
            f"value={self_joint_delta:.10f}"
        )

    offsets = (
        (0.005, 0.0, 0.0),
        (-0.005, 0.0, 0.0),
        (0.0, 0.005, 0.0),
        (0.0, -0.005, 0.0),
        (0.0, 0.0, 0.005),
        (0.0, 0.0, -0.005),
    )

    selected = None

    for offset in offsets:
        target_xyz = [
            current_xyz[index] + offset[index]
            for index in range(3)
        ]

        result = solver.solve_position(
            side,
            target_xyz=target_xyz,
            current_arm_pos=current_arm_pos,
        )

        if not result.success:
            continue

        if result.error_norm > IK_ERROR_LIMIT:
            continue

        side_inactive = inactive_slice(side)

        inactive_delta = max_difference(
            result.arm_pos[side_inactive],
            current_arm_pos[side_inactive],
        )

        if inactive_delta > INACTIVE_ARM_LIMIT:
            continue

        selected = (offset, result)
        break

    if selected is None:
        print(
            f"{side_label}_VIRTUAL_5MM_IK=WARN "
            "reason=no_test_direction_converged"
        )
        return None

    offset, result = selected

    active_delta = max_difference(
        result.arm_pos[active_slice(side)],
        current_arm_pos[active_slice(side)],
    )

    print(
        f"{side_label}_VIRTUAL_5MM_IK=PASS "
        f"offset={offset} "
        f"error_norm={result.error_norm:.10f} "
        f"iterations={result.iterations} "
        f"max_active_joint_delta={active_delta:.10f}"
    )

    return result


def make_synthetic_target(
    current_arm_pos: list[float],
    limits,
):
    target = list(current_arm_pos)

    preferred_indices = (
        3,
        10,
        5,
        12,
    )

    for index in preferred_indices:
        _, lower, upper = limits[index]
        current = target[index]

        for delta in (0.01, -0.01):
            candidate = current + delta

            if lower <= candidate <= upper:
                target[index] = candidate

                print(
                    "RUCKIG_SYNTHETIC_TARGET=true "
                    f"joint={ARM_POS_ORDER[index]} "
                    f"delta={delta}"
                )

                return target

    raise RuntimeError(
        "unable to create an in-limit synthetic target"
    )


def test_ruckig(
    current_positions: list[float],
    current_velocities: list[float],
    target_positions: list[float],
):
    require_finite(
        "ruckig current positions",
        current_positions,
    )
    require_finite(
        "ruckig current velocities",
        current_velocities,
    )
    require_finite(
        "ruckig target positions",
        target_positions,
    )

    if max_abs(current_velocities) > MAX_VELOCITY:
        raise RuntimeError(
            "current joint velocity exceeds configured "
            f"Ruckig maximum: {max_abs(current_velocities)}"
        )

    otg = Ruckig(DOF, CONTROL_DT)
    inp = InputParameter(DOF)
    out = OutputParameter(DOF)

    inp.current_position = list(current_positions)
    inp.current_velocity = list(current_velocities)
    inp.current_acceleration = [0.0] * DOF

    inp.target_position = list(target_positions)
    inp.target_velocity = [0.0] * DOF
    inp.target_acceleration = [0.0] * DOF

    inp.max_velocity = [MAX_VELOCITY] * DOF
    inp.max_acceleration = [MAX_ACCELERATION] * DOF
    inp.max_jerk = [MAX_JERK] * DOF

    final_position = None
    final_step = None

    for step in range(1, MAX_RUCKIG_STEPS + 1):
        result = otg.update(inp, out)

        if result == Result.Finished:
            final_position = list(out.new_position)
            final_step = step
            break

        if result != Result.Working:
            raise RuntimeError(
                f"Ruckig returned error result: {result}"
            )

        out.pass_to_input(inp)

    if final_position is None or final_step is None:
        raise RuntimeError(
            "Ruckig did not finish within step limit"
        )

    final_error = max_difference(
        final_position,
        target_positions,
    )

    if final_error > 1e-6:
        raise RuntimeError(
            f"Ruckig final position error too large: "
            f"{final_error}"
        )

    print(
        "RUCKIG=PASS "
        f"steps={final_step} "
        f"duration={final_step * CONTROL_DT:.6f} "
        f"final_position_error={final_error:.12f}"
    )


def main():
    print("Python:", sys.executable)
    print("NO_COMMAND_PUBLISHED=true")

    response = read_arm_state()

    current_positions, current_velocities = (
        parse_arm_state(response)
    )

    config = X2IKConfig.default_omnipicker()
    solver = X2ArmIKSolver(config)

    print("IK_SOLVER_CREATED=true")
    print("URDF_PATH:", config.urdf_path)

    limits = check_joint_limits(
        solver,
        current_positions,
    )

    left_result = test_side(
        solver,
        ArmSide.LEFT,
        current_positions,
    )

    right_result = test_side(
        solver,
        ArmSide.RIGHT,
        current_positions,
    )

    target_positions = list(current_positions)

    if left_result is not None:
        target_positions[0:7] = (
            list(left_result.active_arm)
        )

    if right_result is not None:
        target_positions[7:14] = (
            list(right_result.active_arm)
        )

    if max_difference(
        target_positions,
        current_positions,
    ) < 1e-9:
        target_positions = make_synthetic_target(
            current_positions,
            limits,
        )

    for index, (_, lower, upper) in enumerate(limits):
        value = target_positions[index]

        if value < lower or value > upper:
            raise RuntimeError(
                "generated Ruckig target outside limits: "
                f"{ARM_POS_ORDER[index]}={value}, "
                f"limits=[{lower}, {upper}]"
            )

    test_ruckig(
        current_positions,
        current_velocities,
        target_positions,
    )

    warnings = (
        left_result is None
        or right_result is None
    )

    if warnings:
        print(
            "OFFLINE_ARM_PIPELINE=PASS_WITH_WARNINGS"
        )
    else:
        print("OFFLINE_ARM_PIPELINE=PASS")

    print("NO_COMMAND_PUBLISHED=true")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(
            f"SELF_TEST_FAIL={type(exc).__name__}: {exc}"
        )
        print("NO_COMMAND_PUBLISHED=true")
        raise
