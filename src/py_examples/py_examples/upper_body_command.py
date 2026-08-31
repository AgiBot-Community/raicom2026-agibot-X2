#!/usr/bin/env python3

import sys
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import UpperBodyCommandArray, MessageHeader


# hand_sub_mode 模式定义
HAND_MODE_NONE = 0           # 无手部控制（仅头/臂）
HAND_MODE_CLAW = 1           # 夹爪模式: hand_pos = [左张合, 右张合]
HAND_MODE_DEXTEROUS = 2      # 灵巧手关节: hand_pos = 20个关节值
HAND_MODE_GESTURE = 3        # 手势模板: hand_pos = [左手势ID, 左张合, 右手势ID, 右张合]


class UpperBodyCommandPublisher(Node):
    def __init__(self):
        super().__init__('upper_body_command_publisher')
        self.publisher = self.create_publisher(
            UpperBodyCommandArray, '/mc/upper_body_command', 10
        )
        self.sequence = 0
        self.get_logger().info('UpperBodyCommand publisher node created.')

    def publish_command(self, hand_sub_mode: int, head_pos: list,
                        arm_pos: list, hand_pos: list):
        msg = UpperBodyCommandArray()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mc_upper_body'
        msg.header.sequence = self.sequence
        self.sequence += 1

        msg.source = 'remote_teleop_pc'
        msg.hand_sub_mode = hand_sub_mode
        msg.head_pos = head_pos
        msg.arm_pos = arm_pos
        msg.hand_pos = hand_pos

        self.publisher.publish(msg)


def build_head_only_msg():
    """头部控制: head_pos = [yaw, pitch] rad"""
    return {
        'hand_sub_mode': HAND_MODE_NONE,
        'head_pos': [0.45, 0.0],
        'arm_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'hand_pos': [],
    }


def build_claw_msg():
    """夹爪模式: hand_pos = [左手张合度[0-1], 右手张合度[0-1]]"""
    return {
        'hand_sub_mode': HAND_MODE_CLAW,
        'head_pos': [0.0, 0.0],
        'arm_pos': [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'hand_pos': [1.0, 0.0],
    }


def build_dexterous_msg():
    """灵巧手关节姿态: hand_pos = [左手10关节, 右手10关节]"""
    return {
        'hand_sub_mode': HAND_MODE_DEXTEROUS,
        'head_pos': [0.0, 0.0],
        'arm_pos': [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'hand_pos': [0.95, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95, 0.95,
                     0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
    }


def build_gesture_msg():
    """手势模板: hand_pos = [左手手势ID, 左手张合度, 右手手势ID, 右手张合度]"""
    return {
        'hand_sub_mode': HAND_MODE_GESTURE,
        'head_pos': [0.0, 0.0],
        'arm_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'hand_pos': [2.0, 1.0, 2.0, 1.0],
    }


MODE_INFO = {
    'HEAD': ('1', 'head only control', build_head_only_msg),
    'CLAW': ('2', 'claw open/close [left, right]', build_claw_msg),
    'DEXTEROUS': ('3', 'dexterous hand joints (20 DOF)', build_dexterous_msg),
    'GESTURE': ('4', 'gesture template [id, open, id, open]', build_gesture_msg),
}


def main(args=None):
    choices = {}
    for k, v in MODE_INFO.items():
        choices[v[0]] = (k, v[2])

    rclpy.init(args=args)
    node = None
    try:
        if len(sys.argv) > 1:
            abbr = sys.argv[1]
        else:
            print('{:<4} - {:<12} : {}'.format('No.', 'mode', 'description'))
            for k, v in MODE_INFO.items():
                print(f'{v[0]:<4} - {k:<12} : {v[1]}')
            abbr = input('Enter number of upper body mode: ')

        entry = choices.get(abbr)
        if not entry:
            raise ValueError(f'Invalid number: {abbr}')

        mode_name, builder = entry
        params = builder()

        node = UpperBodyCommandPublisher()
        node.get_logger().info(
            f'Publishing upper body command: {mode_name} at 50Hz')

        timer = node.create_timer(
            0.02,
            lambda: node.publish_command(**params),
        )

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'Program exited with exception: {e}')

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
