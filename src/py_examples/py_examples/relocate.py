#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import time


class RelocalizationNode(Node):
    def __init__(self):
        super().__init__('relocalization_node')

        # Create publishers
        cmd_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.integrated_command_pub = self.create_publisher(
            String, '/integrated_command', cmd_qos)
        self.relocalization_pose_pub = self.create_publisher(
            Pose, '/relocalization_pose', 10)

        # Create subscriber with BEST_EFFORT QoS
        lidar_loc_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.odometry_sub = self.create_subscription(
            Odometry,
            '/slam/lidar_odom',
            self.odometry_callback,
            qos_profile=lidar_loc_qos
        )

        # Setup timer to publish messages in sequence
        self.success_received = False
        self.timeout_timer = None

        # Start the sequence
        self.publish_sequence()

    def publish_sequence(self):
        # Publish integrated_command
        integrated_command_msg = String()
        integrated_command_msg.data = 'start_relocalization:1774430080403'
        self.integrated_command_pub.publish(integrated_command_msg)
        self.get_logger().info('Published integrated_command')

        # Schedule relocalization_pose publication after delay
        self.relocalization_pose_timer = self.create_timer(
            6.0,
            self.publish_relocalization_pose
        )

    def publish_relocalization_pose(self):
        # Cancel this timer immediately so it only fires once
        self.relocalization_pose_timer.cancel()

        relocalization_pose_msg = Pose()
        relocalization_pose_msg.position.x = 273.0
        relocalization_pose_msg.position.y = 200.0
        relocalization_pose_msg.position.z = 0.0
        relocalization_pose_msg.orientation.x = 0.0
        relocalization_pose_msg.orientation.y = 0.0
        relocalization_pose_msg.orientation.z = 0.0
        relocalization_pose_msg.orientation.w = 1.0

        self.relocalization_pose_pub.publish(relocalization_pose_msg)
        self.get_logger().info('Published relocalization_pose')

        # Start timeout timer (60 seconds)
        self.timeout_timer = self.create_timer(
            60.0,
            self.timeout_callback
        )

        self.get_logger().info('Waiting for robot pose data (timeout: 60s)...')

    def odometry_callback(self, msg):
        if not self.success_received:
            self.success_received = True
            if self.timeout_timer is not None:
                self.timeout_timer.cancel()
            self.get_logger().info('Received odometry data - Relocalization successful!')

    def timeout_callback(self):
        if not self.success_received:
            self.get_logger().error('Timeout reached - Relocalization failed!')
            self.success_received = True


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationNode()
    while rclpy.ok() and not node.success_received:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
