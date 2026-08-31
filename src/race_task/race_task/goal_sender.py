#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('frame_id', 'map')

        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 2.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('target_yaw', 0.0)

        self.declare_parameter('wait_server_timeout_sec', 30.0)
        self.declare_parameter('result_timeout_sec', 300.0)

        self.action_name = self.get_parameter('action_name').value
        self.frame_id = self.get_parameter('frame_id').value

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.target_yaw = float(self.get_parameter('target_yaw').value)

        self.wait_server_timeout_sec = float(
            self.get_parameter('wait_server_timeout_sec').value
        )
        self.result_timeout_sec = float(
            self.get_parameter('result_timeout_sec').value
        )

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name
        )

        self.last_feedback_log_time = 0.0

        self.get_logger().info('goal_sender started.')
        self.get_logger().info(
            f'Target: frame={self.frame_id}, '
            f'x={self.target_x:.3f}, y={self.target_y:.3f}, '
            f'z={self.target_z:.3f}, yaw={self.target_yaw:.3f}'
        )

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return 0.0, 0.0, qz, qw

    def make_goal_msg(self):
        goal_msg = NavigateToPose.Goal()

        now = self.get_clock().now().to_msg()
        goal_msg.pose.header.stamp = now
        goal_msg.pose.header.frame_id = self.frame_id

        goal_msg.pose.pose.position.x = self.target_x
        goal_msg.pose.pose.position.y = self.target_y
        goal_msg.pose.pose.position.z = self.target_z

        qx, qy, qz, qw = self.yaw_to_quaternion(self.target_yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        return goal_msg

    def feedback_callback(self, feedback_msg):
        now = time.time()
        if now - self.last_feedback_log_time < 1.0:
            return

        feedback = feedback_msg.feedback
        self.last_feedback_log_time = now

        try:
            distance_remaining = feedback.distance_remaining
            self.get_logger().info(
                f'Navigation feedback: distance_remaining={distance_remaining:.3f} m'
            )
        except Exception:
            self.get_logger().info('Navigation feedback received.')

    def status_to_text(self, status):
        table = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return table.get(status, f'UNRECOGNIZED({status})')

    def send_goal(self):
        self.get_logger().info(
            f'Waiting for Nav2 action server: {self.action_name}'
        )

        server_ready = self.action_client.wait_for_server(
            timeout_sec=self.wait_server_timeout_sec
        )

        if not server_ready:
            self.get_logger().error(
                f'Nav2 action server not available: {self.action_name}'
            )
            return False

        goal_msg = self.make_goal_msg()

        self.get_logger().info('Sending navigation goal...')
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error('Failed to send goal: goal_handle is None.')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected.')
            return False

        self.get_logger().info('Navigation goal accepted.')

        result_future = goal_handle.get_result_async()
        start_time = time.time()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if result_future.done():
                result = result_future.result()
                status_text = self.status_to_text(result.status)
                self.get_logger().info(f'Navigation result: {status_text}')

                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info('Goal reached successfully.')
                    return True

                self.get_logger().warn('Goal did not succeed.')
                return False

            if time.time() - start_time > self.result_timeout_sec:
                self.get_logger().warn('Navigation timeout. Canceling goal...')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return False

        return False


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()

    try:
        ok = node.send_goal()
        if ok:
            node.get_logger().info('goal_sender finished successfully.')
        else:
            node.get_logger().warn('goal_sender finished with failure.')
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
