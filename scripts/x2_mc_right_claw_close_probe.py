#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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


RIGHT_CLOSE = 0.0


def read_hand(node):
    node.latest_hand_state = None

    hand = node.wait_for_claw_state(
        timeout_sec=5.0
    )

    return np.asarray(
        hand,
        dtype=float,
    )


def main():

    rclpy.init()

    node = XYZClosedLoopNode()

    pump = None
    in_urs = False

    try:
        # ---------------------------------------
        # 1. 读取当前真实机械臂状态
        # ---------------------------------------

        arm, head, _ = node.read_state()

        arm = np.asarray(
            arm,
            dtype=float,
        )

        head = np.asarray(
            head,
            dtype=float,
        )

        hand = read_hand(node)

        print(
            "=== MC右夹爪闭合诊断 ==="
        )

        print(
            "INITIAL_HAND="
            f"[{hand[0]:.6f}, {hand[1]:.6f}]"
        )

        target_hand = hand.copy()

        # hand_pos顺序：
        # [left, right]
        target_hand[1] = RIGHT_CLOSE

        print(
            "TARGET_HAND="
            f"[{target_hand[0]:.6f}, "
            f"{target_hand[1]:.6f}]"
        )

        print(
            "RIGHT_TARGET=0.000000"
        )

        # ---------------------------------------
        # 2. ROS graph检查
        # ---------------------------------------

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
            f"UPPER_BODY_SUBSCRIBERS={subscribers}"
        )

        print(
            f"UPPER_BODY_PUBLISHERS={publishers}"
        )

        if subscribers < 1:
            raise RuntimeError(
                "/mc/upper_body_command "
                "没有订阅者"
            )

        # 本节点创建后自身就是1个publisher
        if publishers != 1:
            raise RuntimeError(
                "存在额外UpperBodyCommand publisher，"
                f"当前={publishers}"
            )

        # ---------------------------------------
        # 3. 先持续发布当前状态
        # ---------------------------------------

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

        # ---------------------------------------
        # 4. 进入比赛机上肢分离模式
        # ---------------------------------------

        set_mode_with_pump(
            node,
            pump,
            "UPPERBODY_REMOTE_SPLIT",
        )

        in_urs = True

        pump.hold(2.0)

        node.validate_hal_takeover(
            arm
        )

        print(
            "URS_TAKEOVER=PASS"
        )

        # ---------------------------------------
        # 5. 唯一关键动作：
        #    通过MC发送 right hand = 0
        # ---------------------------------------

        print(
            "RIGHT_CLOSE_COMMAND_START=true"
        )

        print(
            "REAL_ROBOT_MOTION=true"
        )

        pump.set_command(
            arm,
            hand=target_hand,
        )

        # 给夹爪5秒时间执行
        pump.hold(5.0)

        # ---------------------------------------
        # 6. 读取真实反馈
        # ---------------------------------------

        hand_after = read_hand(node)

        print(
            "HAND_AFTER_5S="
            f"[{hand_after[0]:.6f}, "
            f"{hand_after[1]:.6f}]"
        )

        print(
            "RIGHT_AFTER_5S="
            f"{hand_after[1]:.6f}"
        )

        print(
            "RIGHT_CHANGE="
            f"{hand_after[1] - hand[1]:+.6f}"
        )

        print(
            "COMMAND_PUMP_MAX_GAP_MS="
            f"{pump.max_gap_sec * 1000:.3f}"
        )

        print(
            "================================================"
        )

        print(
            "保持测试中：现在请在另一个终端查看 "
            "Hand HAL command 和 hand state。"
        )

        print(
            "不要关闭本程序。"
        )

        print(
            "按 Ctrl+C 后才退出测试。"
        )

        print(
            "================================================"
        )

        # ---------------------------------------
        # 7. 一直保持 right=0
        # ---------------------------------------

        while rclpy.ok():

            pump.hold(1.0)

            state = read_hand(node)

            print(
                "RIGHT_HOLD_STATE="
                f"{state[1]:.6f}"
            )

            time.sleep(1.0)

    except KeyboardInterrupt:

        print(
            "\nUSER_INTERRUPT=true"
        )

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
            in_urs
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
                    "RESTORE_STAND_DEFAULT=PASS"
                )

            except Exception as exc:
                print(
                    "RESTORE_STAND_DEFAULT=FAIL"
                )

                print(
                    f"REASON={exc}"
                )

        if pump is not None:
            pump.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
