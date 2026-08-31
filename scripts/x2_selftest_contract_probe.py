#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from typing import Any

import rclpy

from aimdk_msgs.msg import (
    CommonRequest,
    CommonResponse,
    JointState,
)
from aimdk_msgs.srv import GetAllJointState

from x2_ik_sdk import X2ArmIKSolver, X2IKConfig
from x2_ik_sdk.config import ARM_POS_ORDER, ArmSide


TARGET_SERVICE_TYPE = "aimdk_msgs/srv/GetAllJointState"


def print_message_fields(title: str, message: Any) -> None:
    print(f"\n========== {title} ==========")

    fields = message.get_fields_and_field_types()

    for name, field_type in fields.items():
        value = getattr(message, name)
        print(
            f"{name}: type={field_type}, "
            f"default={value!r}"
        )


def describe_result(title: str, result: Any) -> None:
    print(f"\n========== {title} ==========")
    print("RESULT_TYPE:", type(result))
    print("RESULT_REPR:", repr(result))

    result_dict = getattr(result, "__dict__", None)

    if isinstance(result_dict, dict):
        print("RESULT_DICT:")

        for name, value in sorted(result_dict.items()):
            print(f"  {name}={value!r}")

    print("PUBLIC_ATTRIBUTES:")

    for name in sorted(dir(result)):
        if name.startswith("_"):
            continue

        try:
            value = getattr(result, name)
        except Exception as exc:
            print(f"  {name}=<read error: {exc!r}>")
            continue

        if callable(value):
            continue

        print(f"  {name}={value!r}")


def main() -> None:
    print("Python:", sys.executable)

    print_message_fields(
        "CommonRequest",
        CommonRequest(),
    )

    print_message_fields(
        "CommonResponse",
        CommonResponse(),
    )

    print_message_fields(
        "JointState",
        JointState(),
    )

    print_message_fields(
        "GetAllJointState.Request",
        GetAllJointState.Request(),
    )

    print_message_fields(
        "GetAllJointState.Response",
        GetAllJointState.Response(),
    )

    print("\n========== ROS SERVICE DISCOVERY ==========")

    rclpy.init()
    node = rclpy.create_node(
        "x2_selftest_contract_probe"
    )

    try:
        matches = []

        for service_name, service_types in (
            node.get_service_names_and_types()
        ):
            if TARGET_SERVICE_TYPE in service_types:
                matches.append(service_name)

        print("SERVICE_TYPE:", TARGET_SERVICE_TYPE)
        print("MATCHING_SERVICE_COUNT:", len(matches))

        for service_name in sorted(matches):
            print("MATCHING_SERVICE:", service_name)

    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("\n========== OFFLINE FK/IK CONTRACT ==========")

    config = X2IKConfig.default_omnipicker()
    solver = X2ArmIKSolver(config)
    ready = solver.ready_arm_pos()

    print("ARM_POS_ORDER_LENGTH:", len(ARM_POS_ORDER))
    print("READY_LENGTH:", len(ready))

    limits = solver.joint_limits_for_arm_pos()

    print("LIMIT_COUNT:", len(limits))

    for index, item in enumerate(limits):
        print(f"LIMIT_{index:02d}: {item!r}")

    for side in (ArmSide.LEFT, ArmSide.RIGHT):
        side_name = side.value.upper()

        xyz = solver.fk_xyz(
            side,
            current_arm_pos=ready,
        )

        rpy = solver.fk_rpy(
            side,
            current_arm_pos=ready,
        )

        print(f"\n{side_name}_READY_XYZ:", xyz)
        print(f"{side_name}_READY_RPY:", rpy)

        result = solver.solve_position(
            side,
            target_xyz=xyz,
            current_arm_pos=ready,
        )

        describe_result(
            f"{side_name} IKResult",
            result,
        )

    print("\nCONTRACT_PROBE=PASS")
    print("SERVICE_CALLED=false")
    print("NO_COMMAND_PUBLISHED=true")


if __name__ == "__main__":
    main()
