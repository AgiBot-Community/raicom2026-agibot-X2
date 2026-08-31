#!/usr/bin/env python3

import time
import threading
import rclpy
import rclpy.logging
from rclpy.node import Node

from aimdk_msgs.msg import PncTaskRequest, MessageHeader
from geometry_msgs.msg import PoseStamped

# task_request enum
TASK_REQUEST_START = 1
TASK_REQUEST_STOP = 2
TASK_REQUEST_PAUSE = 3
TASK_REQUEST_RESUME = 4


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_example')
        self.publisher = self.create_publisher(
            PncTaskRequest, '/aima/te/pnc_task_request', 10
        )

        now_ms = int(time.time() * 1000)
        self.task_id = now_ms
        self.map_id = 0
        self.timer = None
        self.current_msg = None

        self.get_logger().info(
            f'Navigation node started, task_id={self.task_id}')

    def send_start(self, map_id, x, y, z, ox, oy, oz, ow, radius, pnc_mode, max_speed):
        self.map_id = map_id
        msg = PncTaskRequest()
        msg.header = self._make_header()
        msg.task_type = 2
        msg.task_request = TASK_REQUEST_START
        msg.pnc_mode = pnc_mode
        msg.max_forward_speed = max_speed
        msg.task_id = self.task_id
        msg.map_id = map_id
        msg.target_pose_radius = radius
        msg.reserve_info = ['\x00'] * 64

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.x = ox
        target.pose.orientation.y = oy
        target.pose.orientation.z = oz
        target.pose.orientation.w = ow
        msg.target_pose = target

        self.get_logger().info(
            f'[Start Navigation] task_id={self.task_id}, map_id={map_id}, '
            f'target=({x:.2f}, {y:.2f}, {z:.2f}), radius={radius:.2f}')
        self._start_publishing(msg)

    def send_pause(self):
        msg = self._make_simple_request(TASK_REQUEST_PAUSE, 0)
        self.get_logger().info(f'[Pause Navigation] task_id={self.task_id}')
        self._start_publishing(msg)

    def send_resume(self):
        msg = self._make_simple_request(TASK_REQUEST_RESUME, self.map_id)
        self.get_logger().info(
            f'[Resume Navigation] task_id={self.task_id}, map_id={self.map_id}')
        self._start_publishing(msg)

    def send_stop(self):
        msg = self._make_simple_request(TASK_REQUEST_STOP, self.map_id)
        self.get_logger().info(
            f'[Stop Navigation] task_id={self.task_id}, map_id={self.map_id}')
        self._start_publishing(msg)

    def stop_publishing(self):
        if self.timer:
            self.timer.cancel()
            self.timer = None
            self.get_logger().info('Stopped continuous publishing')

    def _start_publishing(self, msg):
        self.current_msg = msg
        if self.timer:
            self.timer.cancel()
        self.publisher.publish(self.current_msg)

        def timer_callback():
            self.current_msg.header.stamp = self.get_clock().now().to_msg()
            self.current_msg.header.meas_stamp = self.get_clock().now().to_msg()
            self.current_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.current_msg)

        self.timer = self.create_timer(1.0, timer_callback)
        self.get_logger().info('Continuous publishing at 1Hz')

    def _make_header(self):
        header = MessageHeader()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.meas_stamp = now
        header.frame_id = 'map'
        header.sequence = 0
        return header

    def _make_simple_request(self, task_request, map_id):
        msg = PncTaskRequest()
        msg.header = self._make_header()
        msg.task_type = 2
        msg.task_request = task_request
        msg.pnc_mode = 0
        msg.max_forward_speed = 0.5
        msg.task_id = self.task_id
        msg.map_id = map_id
        msg.target_pose_radius = 0.0
        msg.reserve_info = ['\x00'] * 64
        msg.target_pose.pose.orientation.w = 1.0
        return msg


def input_value(prompt, default_val):
    """带默认值的输入，直接回车使用默认值"""
    line = input(f'{prompt} [{default_val}]: ').strip()
    if not line:
        return default_val
    return type(default_val)(line)


def print_menu(task_id):
    print(f'\n========================================')
    print(f'  Navigation Control  (task_id={task_id})')
    print(f'========================================')
    print(f'  1. Start Navigation')
    print(f'  2. Pause Navigation')
    print(f'  3. Resume Navigation')
    print(f'  4. Stop Navigation')
    print(f'  q. Quit')
    print(f'========================================')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    node.get_logger().info('Waiting for node communication ready (5s)...')
    time.sleep(5)

    try:
        while rclpy.ok():
            print_menu(node.task_id)
            choice = input('Select an option: ').strip()

            if choice == '1':
                print('\n--- Start Navigation Parameters ---')
                map_id = input_value('Map ID (map_id)', 1773113429735)
                x = input_value('Target x', 1.0)
                y = input_value('Target y', 2.0)
                z = input_value('Target z', 0.0)
                ox = input_value('Orientation x', 0.0)
                oy = input_value('Orientation y', 0.0)
                oz = input_value('Orientation z', 0.0)
                ow = input_value('Orientation w', 1.0)
                radius = input_value('Target pose radius', 0.5)
                pnc_mode = input_value('pnc_mode', 0)
                max_speed = input_value('Max forward speed', 0.5)
                node.send_start(map_id, x, y, z, ox, oy, oz, ow,
                                radius, pnc_mode, max_speed)
            elif choice == '2':
                node.send_pause()
            elif choice == '3':
                node.send_resume()
            elif choice == '4':
                node.send_stop()
            elif choice in ('q', 'Q'):
                node.stop_publishing()
                print('Exiting navigation control.')
                break
            else:
                print('Invalid input, please try again.')
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger('main').error(
            f'Program exited with exception: {e}')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
