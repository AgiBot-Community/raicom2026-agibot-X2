#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist
from aimdk_msgs.msg import McLocomotionVelocity, MessageHeader
from aimdk_msgs.srv import SetMcInputSource


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/aima/mc/locomotion/velocity')
        self.declare_parameter('source', 'node')

        self.declare_parameter('register_input_source', True)
        self.declare_parameter('input_source_priority', 40)
        self.declare_parameter('input_source_timeout', 1000)

        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        self.declare_parameter('max_forward_velocity', 0.5)
        self.declare_parameter('max_backward_velocity', 0.3)
        self.declare_parameter('max_lateral_velocity', 0.0)
        self.declare_parameter('max_angular_velocity', 0.6)

        # 官方例程里的 MC 启动阈值：
        # forward/lateral: 0 或 ±(0.2~1.0)
        # angular: 0 或 ±(0.1~1.0)
        self.declare_parameter('min_forward_velocity', 0.2)
        self.declare_parameter('min_lateral_velocity', 0.2)
        self.declare_parameter('min_angular_velocity', 0.1)
        self.declare_parameter('deadband', 0.005)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.source = self.get_parameter('source').value

        self.register_input_source_enabled = bool(
            self.get_parameter('register_input_source').value
        )
        self.input_source_priority = int(
            self.get_parameter('input_source_priority').value
        )
        self.input_source_timeout = int(
            self.get_parameter('input_source_timeout').value
        )

        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)

        self.max_forward_velocity = float(
            self.get_parameter('max_forward_velocity').value
        )
        self.max_backward_velocity = float(
            self.get_parameter('max_backward_velocity').value
        )
        self.max_lateral_velocity = float(
            self.get_parameter('max_lateral_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value
        )

        self.min_forward_velocity = float(
            self.get_parameter('min_forward_velocity').value
        )
        self.min_lateral_velocity = float(
            self.get_parameter('min_lateral_velocity').value
        )
        self.min_angular_velocity = float(
            self.get_parameter('min_angular_velocity').value
        )
        self.deadband = float(self.get_parameter('deadband').value)

        self.forward_velocity = 0.0
        self.lateral_velocity = 0.0
        self.angular_velocity = 0.0
        self.last_cmd_time = None

        self.publisher = self.create_publisher(
            McLocomotionVelocity,
            self.output_topic,
            10
        )

        self.subscriber = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.input_source_client = self.create_client(
            SetMcInputSource,
            '/aimdk_5Fmsgs/srv/SetMcInputSource'
        )

        if self.register_input_source_enabled:
            self.register_input_source()

        timer_period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.publish_velocity)

        self.get_logger().info('cmd_vel_bridge started.')
        self.get_logger().info(f'Input : {self.cmd_vel_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')
        self.get_logger().info(f'Source: {self.source}')
        self.get_logger().info(f'Rate  : {self.publish_hz} Hz')

    def register_input_source(self):
        self.get_logger().info('Registering MC input source...')

        timeout_sec = 8.0
        start = self.get_clock().now().nanoseconds / 1e9

        while not self.input_source_client.wait_for_service(timeout_sec=1.0):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - start > timeout_sec:
                self.get_logger().error('SetMcInputSource service timeout.')
                return False
            self.get_logger().info('Waiting for SetMcInputSource service...')

        req = SetMcInputSource.Request()
        req.action.value = 1001
        req.input_source.name = self.source
        req.input_source.priority = self.input_source_priority
        req.input_source.timeout = self.input_source_timeout

        for i in range(8):
            req.request.header.stamp = self.get_clock().now().to_msg()
            future = self.input_source_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.25)

            if future.done():
                break

            self.get_logger().info(f'Trying to register input source... [{i}]')

        if not future.done():
            self.get_logger().error('Input source registration failed or timed out.')
            return False

        try:
            response = future.result()
            self.get_logger().info(
                f'Input source registration response: '
                f'state={response.response.state.value}, '
                f'task_id={response.response.task_id}'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'Input source service exception: {e}')
            return False

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def apply_threshold(self, value, min_abs, max_abs):
        if abs(value) < self.deadband:
            return 0.0

        sign = 1.0 if value > 0 else -1.0
        abs_value = abs(value)

        abs_value = min(abs_value, max_abs)

        if abs_value < min_abs:
            abs_value = min_abs

        return sign * abs_value

    def cmd_vel_callback(self, msg: Twist):
        raw_forward = float(msg.linear.x)
        raw_lateral = float(msg.linear.y)
        raw_angular = float(msg.angular.z)

        # X2 前进和后退限幅分开处理
        if raw_forward >= 0.0:
            max_forward = self.max_forward_velocity
        else:
            max_forward = self.max_backward_velocity

        self.forward_velocity = self.apply_threshold(
            raw_forward,
            self.min_forward_velocity,
            max_forward
        )

        if self.max_lateral_velocity <= 0.0:
            self.lateral_velocity = 0.0
        else:
            self.lateral_velocity = self.apply_threshold(
                raw_lateral,
                self.min_lateral_velocity,
                self.max_lateral_velocity
            )

        self.angular_velocity = self.apply_threshold(
            raw_angular,
            self.min_angular_velocity,
            self.max_angular_velocity
        )

        self.last_cmd_time = self.get_clock().now()

    def publish_velocity(self):
        now = self.get_clock().now()

        if self.last_cmd_time is None:
            forward = 0.0
            lateral = 0.0
            angular = 0.0
        else:
            age = (now - self.last_cmd_time).nanoseconds / 1e9
            if age > self.cmd_timeout_sec:
                forward = 0.0
                lateral = 0.0
                angular = 0.0
            else:
                forward = self.forward_velocity
                lateral = self.lateral_velocity
                angular = self.angular_velocity

        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = now.to_msg()
        msg.source = self.source
        msg.forward_velocity = float(forward)
        msg.lateral_velocity = float(lateral)
        msg.angular_velocity = float(angular)

        self.publisher.publish(msg)

    def stop_robot(self):
        msg = McLocomotionVelocity()
        msg.header = MessageHeader()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.source = self.source
        msg.forward_velocity = 0.0
        msg.lateral_velocity = 0.0
        msg.angular_velocity = 0.0

        for _ in range(10):
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_robot()
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
