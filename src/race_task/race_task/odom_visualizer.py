#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class OdomVisualizer(Node):
    def __init__(self):
        super().__init__('odom_visualizer')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('marker_topic', '/race_task/robot_marker')
        self.declare_parameter('path_topic', '/race_task/odom_path')
        self.declare_parameter('path_keep', 5000)
        self.declare_parameter('arrow_length', 1.2)
        self.declare_parameter('arrow_width', 0.25)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.path_keep = int(self.get_parameter('path_keep').value)
        self.arrow_length = float(self.get_parameter('arrow_length').value)
        self.arrow_width = float(self.get_parameter('arrow_width').value)

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.path_msg = Path()

        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info('odom_visualizer started.')
        self.get_logger().info(f'Subscribe: {self.odom_topic}')
        self.get_logger().info(f'Publish marker: {self.marker_topic}')
        self.get_logger().info(f'Publish path: {self.path_topic}')

    def odom_callback(self, msg: Odometry):
        frame_id = msg.header.frame_id
        if not frame_id:
            frame_id = 'odom'

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = frame_id
        marker.ns = 'race_task_robot'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose = msg.pose.pose

        marker.scale.x = self.arrow_length
        marker.scale.y = self.arrow_width
        marker.scale.z = self.arrow_width

        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path_msg.header = msg.header
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > self.path_keep:
            self.path_msg.poses = self.path_msg.poses[-self.path_keep:]

        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
