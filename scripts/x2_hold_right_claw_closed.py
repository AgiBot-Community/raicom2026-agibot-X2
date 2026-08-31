#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys
import time

import numpy as np
import rclpy

from x2_safe_joint_microtest import COMMAND_TOPIC

from x2_competition_move_xyz_closed_loop import (
    XYZClosedLoopNode,
)

from x2_competition_xyz_persistent_stream_test import (
    PersistentCommandPump,
    set_mode_with_pump,
)


RIGHT_CLAW_CLOSE = 0.0


def read_fresh_hand(node):
    node.latest_hand_state = None

    return np.asarray(
        node.wait_for_claw_state(
            timeout_sec=5.0
        ),
        dtype=float,
    )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--execute",
        action="store_true",
        help="真正执行右夹爪闭合",
    )

    args = parser.parse_args()

    rclpy.init()

    node = XYZClosedLoopNode()

    pump = None
    switched_to_urs = False

    try:
        # =========================================
        # 1. 读取当前机械臂
        # =========================================

        arm, head, _ = node.read_state()

        arm = np.asarray(
            arm,
            dtype=float,
        )

        head = np.asarray(
            head,
            dtype=float,
        )

        # =========================================
        # 2. 读取真实夹爪
        # =========================================

        hand = read_fresh_hand(
            node
        )

        target_hand = hand.copy()

        # [left, right]
        target_hand[1] = RIGHT_CLAW_CLOSE

        print(
            "=== X2右夹爪持续闭合 ==="
        )

        print(
            f"EXECUTE={str(args.execute).lower()}"
        )

        print(
            "ARM_COMMAND=HOLD_CURRENT"
        )

        print(
            "INITIAL_HAND="
            f"[{hand[0]:.6f}, "
            f"{hand[1]:.6f}]"
        )

        print(
            "TARGET_HAND="
            f"[{target_hand[0]:.6f}, "
            f"{target_hand[1]:.6f}]"
        )

        print(
            "RIGHT_CLAW_TARGET=0.000000"
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
        # 3. 检查UpperBody ROS图
        # =========================================

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

        print(
            f"COMMAND_SUBSCRIBERS={subscribers}"
        )

        print(
            f"COMMAND_PUBLISHERS={publishers}"
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command没有订阅者"
            )

        # 这个Node本身已经创建1个publisher。
        if publishers > 1:
            raise RuntimeError(
                "存在其他UpperBodyCommand publisher，"
                "先停止旧机械臂节点"
            )

        print(
            "REAL_ROBOT_MOTION=true"
        )

        # =========================================
        # 4. 先用当前姿态启动持续50Hz
        # =========================================

        pump = PersistentCommandPump(
            node,
            arm,
            head,
            hand,
        )

        pump.start()

        pump.hold(2.0)

        print(
            "COMMAND_PUMP_STARTED=true"
        )

        # =========================================
        # 5. 切换比赛机上肢分离模式
        # =========================================

        set_mode_with_pump(
            node,
            pump,
            "UPPERBODY_REMOTE_SPLIT",
        )

        switched_to_urs = True

        pump.hold(2.0)

        node.validate_hal_takeover(
            arm
        )

        print(
            "TAKEOVER_VALIDATED=true"
        )

        # =========================================
        # 6. 真正关闭右夹爪
        # =========================================

        print(
            "RIGHT_CLAW_CLOSE_START=true"
        )

        pump.set_command(
            arm,
            hand=target_hand,
        )

        pump.hold(3.0)

        closed = read_fresh_hand(
            node
        )

        print(
            "CLOSED_HAND_STATE="
            f"[{closed[0]:.6f}, "
            f"{closed[1]:.6f}]"
        )

        print(
            "RIGHT_CLAW_STATE="
            f"{closed[1]:.6f}"
        )

        # =========================================
        # 7. 持续保持
        # =========================================

        print(
            "RIGHT_CLAW_HOLDING=true"
        )

        print(
            "IMPORTANT=保持本程序运行，不要Ctrl+C"
        )

        print(
            "IMPORTANT=Ctrl+C后将恢复STAND_DEFAULT，"
            "夹爪可能再次被MC打开"
        )

        last_report = time.monotonic()

        while rclpy.ok():

            # pump自身会50Hz不断发布最后目标。
            pump.hold(1.0)

            now = time.monotonic()

            if (
                now - last_report
                >= 5.0
            ):

                state = read_fresh_hand(
                    node
                )

                print(
                    "RIGHT_CLAW_HOLD_STATE="
                    f"{state[1]:.6f}"
                )

                print(
                    "COMMAND_PUMP_MAX_GAP_MS="
                    f"{pump.max_gap_sec * 1000:.3f}"
                )

                last_report = now

        return 0

    except KeyboardInterrupt:

        print(
            "\nUSER_INTERRUPT=true"
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
        ):

            try:

                print(
                    "RESTORE_STAND_DEFAULT=true"
                )

                set_mode_with_pump(
                    node,
                    pump,
                    "STAND_DEFAULT",
                )

                print(
                    "RESTORE_RESULT=PASS"
                )

                print(
                    "WARNING=STAND_DEFAULT后夹爪"
                    "可能重新变为打开状态"
                )

            except Exception as exc:

                print(
                    "RESTORE_RESULT=FAIL"
                )

                print(
                    f"RESTORE_REASON={exc}"
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
