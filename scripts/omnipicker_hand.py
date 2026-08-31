#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OmniPicker 双夹爪控制——后台守护节点版
"""

import argparse
import glob
import os
import subprocess
import sys

COMMAND_TOPIC = "/aima/hal/joint/hand/command"
LEFT_JOINT_NAME = "left_claw_joint"
RIGHT_JOINT_NAME = "right_claw_joint"
PUBLISH_FREQUENCY_HZ = 50.0
_REEXEC_FLAG = "_OMNIPICKER_STUDENT_REEXEC"


def load_ros_environment():
    setup_files = sorted(glob.glob("/opt/ros/*/setup.bash"))
    aimdk_setup = os.path.expanduser("~/aimdk/install/setup.bash")

    commands = []

    if setup_files:
        commands.append("source " + setup_files[0])

    if os.path.exists(aimdk_setup):
        commands.append("source " + aimdk_setup)

    if not commands:
        return

    result = subprocess.run(
        ["bash", "-c", " && ".join(commands) + " && env"],
        capture_output=True,
        text=True,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError("ROS 2/AimDK 环境加载失败")

    for line in result.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            os.environ[key] = value

    for path in os.environ.get("PYTHONPATH", "").split(":"):
        if path and path not in sys.path:
            sys.path.insert(0, path)


load_ros_environment()


try:
    import rclpy

    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    from std_msgs.msg import String

    from aimdk_msgs.msg import (
        HandCommand,
        HandCommandArray,
        HandType,
        MessageHeader,
    )

except ImportError as exc:

    if not os.environ.get(_REEXEC_FLAG):
        os.environ[_REEXEC_FLAG] = "1"
        os.execv(
            sys.executable,
            [sys.executable] + sys.argv,
        )

    print(
        "无法导入 ROS 2 或 AimDK Python 类型：",
        exc,
    )

    sys.exit(2)


def create_hand_command(
    joint_name,
    target_position,
):
    command = HandCommand()

    command.name = joint_name
    command.position = float(target_position)

    command.velocity = 1.0
    command.acceleration = 1.0
    command.deceleration = 1.0
    command.effort = 1.0

    return command


def build_hand_message(
    hand,
    target_position,
):
    msg = HandCommandArray()

    msg.header = MessageHeader()

    msg.left_hand_type = HandType(
        value=2 if hand == "left" else 0
    )

    msg.right_hand_type = HandType(
        value=2 if hand == "right" else 0
    )

    msg.left_hands = (
        [
            create_hand_command(
                LEFT_JOINT_NAME,
                target_position,
            )
        ]
        if hand == "left"
        else []
    )

    msg.right_hands = (
        [
            create_hand_command(
                RIGHT_JOINT_NAME,
                target_position,
            )
        ]
        if hand == "right"
        else []
    )

    return msg


class OmniPickerDaemonNode(Node):

    def __init__(
        self,
        target_hand,
        initial_position,
    ):
        super().__init__(
            "omnipicker_gripper_daemon"
        )

        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = (
            self.create_publisher(
                HandCommandArray,
                COMMAND_TOPIC,
                command_qos,
            )
        )

        self.target_hand = target_hand
        self.current_position = initial_position

        self.timer = (
            self.create_timer(
                1.0 / PUBLISH_FREQUENCY_HZ,
                self.timer_callback,
            )
        )

        self.subscriber = (
            self.create_subscription(
                String,
                "/gripper_cmd",
                self.cmd_callback,
                10,
            )
        )

        state_str = (
            "张开"
            if self.current_position == 1.0
            else "闭合"
        )

        self.get_logger().info(
            "夹爪守护节点已启动！"
            f"目标: {self.target_hand}, "
            f"初始状态: {state_str}"
        )

        self.get_logger().info(
            "监听 /gripper_cmd ："
            "open / close"
        )

    def timer_callback(self):

        msg = build_hand_message(
            self.target_hand,
            self.current_position,
        )

        self.publisher.publish(msg)

    def cmd_callback(self, msg):

        command = (
            msg.data
            .strip()
            .lower()
        )

        if command == "open":

            self.current_position = 1.0

            self.get_logger().info(
                "收到指令：张开夹爪"
            )

        elif command == "close":

            self.current_position = 0.0

            self.get_logger().info(
                "收到指令：闭合夹爪"
            )

        else:

            self.get_logger().warning(
                f"未知夹爪命令: {command}"
            )


def parse_arguments():

    parser = argparse.ArgumentParser(
        description="OmniPicker 双夹爪控制守护节点"
    )

    parser.add_argument(
        "--publish",
        action="store_true",
    )

    parser.add_argument(
        "action",
        choices=("open", "close"),
    )

    parser.add_argument(
        "hand",
        choices=("left", "right"),
    )

    return parser.parse_args()


def main(args=None):

    parsed_args = parse_arguments()

    if not parsed_args.publish:

        print(
            "未指定 --publish，程序退出。"
        )

        return

    rclpy.init(args=args)

    initial_pos = (
        1.0
        if parsed_args.action == "open"
        else 0.0
    )

    node = OmniPickerDaemonNode(
        parsed_args.hand,
        initial_pos,
    )

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print(
            "\n已停止夹爪守护节点。"
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
