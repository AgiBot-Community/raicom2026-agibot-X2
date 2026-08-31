#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
X2 IK SDK只读接口探测。

安全属性：
- 不创建Publisher
- 不发布机械臂或夹爪命令
- 不调用SetMcAction
- 不调用SetMcInputSource
- 不调用任何运动服务
"""

import inspect
import sys

from aimdk_msgs.srv import GetAllJointState
from x2_ik_sdk import X2ArmIKSolver, X2IKConfig
from x2_ik_sdk.config import ARM_POS_ORDER, ArmSide


def safe_signature(obj):
    try:
        return str(inspect.signature(obj))
    except Exception as exc:
        return f"<signature unavailable: {type(exc).__name__}>"


def print_object_fields(title, obj):
    print(f"\n========== {title} ==========")

    try:
        fields = obj.get_fields_and_field_types()
    except Exception as exc:
        print("get_fields_and_field_types unavailable:", repr(exc))
        return

    for name, field_type in fields.items():
        print(f"{name}: {field_type}")


def main():
    print("Python:", sys.executable)

    print_object_fields(
        "GetAllJointState.Request",
        GetAllJointState.Request(),
    )

    print_object_fields(
        "GetAllJointState.Response",
        GetAllJointState.Response(),
    )

    print("\n========== ARM CONFIG ==========")
    print("ARM_POS_ORDER_LENGTH:", len(ARM_POS_ORDER))

    for index, name in enumerate(ARM_POS_ORDER):
        print(f"{index:02d}: {name}")

    print("ArmSide:", list(ArmSide))

    print("\n========== X2IKConfig ==========")

    config = X2IKConfig.default_omnipicker()

    print("config type:", type(config))
    print("config repr:", repr(config))

    config_dict = getattr(config, "__dict__", None)

    if isinstance(config_dict, dict):
        for name, value in sorted(config_dict.items()):
            print(f"{name}: {value!r}")
    else:
        print("__dict__: unavailable")

    print("\n========== X2ArmIKSolver ==========")

    solver = X2ArmIKSolver(config)

    print("solver type:", type(solver))

    public_names = [
        name
        for name in dir(solver)
        if not name.startswith("_")
    ]

    for name in public_names:
        try:
            value = getattr(solver, name)
        except Exception as exc:
            print(f"{name}: <attribute error: {exc!r}>")
            continue

        if callable(value):
            print(f"{name}{safe_signature(value)}")
        elif name in {
            "model",
            "data",
            "config",
            "left_ee_frame",
            "right_ee_frame",
        }:
            print(f"{name}: {type(value)}")

    print("\n========== READY ARM POSITION ==========")

    ready = solver.ready_arm_pos()

    print("READY_TYPE:", type(ready))
    print("READY_LENGTH:", len(ready))
    print("READY_VALUES:", list(ready))

    if len(ready) != 14:
        raise RuntimeError(
            f"expected 14 ready-arm values, got {len(ready)}"
        )

    print("\nSDK_API_PROBE=PASS")
    print("NO_COMMAND_PUBLISHED=true")


if __name__ == "__main__":
    main()
