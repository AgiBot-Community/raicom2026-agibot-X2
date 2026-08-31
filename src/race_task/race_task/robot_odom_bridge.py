#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class RobotOdomBridge(Node):
    def __init__(self):
        super().__init__('robot_odom_bridge')

        self.declare_parameter('input_odom_topic', '/aima/mc/leg_odometry')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'leg_odom')
        self.declare_parameter('base_frame_id', 'lidar_imu_chest_front')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_hz', 30.0)

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.latest_msg = None
        self.received_count = 0

        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub = self.create_publisher(Odometry, self.output_odom_topic, output_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 同时尝试 4 种 QoS，防止机器人端 QoS 特殊导致收不到
        qos_profiles = [
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        ]

        self.subs = []
        for i, qos in enumerate(qos_profiles):
            sub = self.create_subscription(
                Odometry,
                self.input_odom_topic,
                lambda msg, qos_id=i: self.odom_callback(msg, qos_id),
                qos
            )
            self.subs.append(sub)

        timer_period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('robot_odom_bridge started.')
        self.get_logger().info(f'Input : {self.input_odom_topic}')
        self.get_logger().info(f'Output: {self.output_odom_topic}')
        self.get_logger().info(f'TF    : {self.odom_frame_id} -> {self.base_frame_id}')
        self.get_logger().info(f'Rate  : {self.publish_hz} Hz')
        self.get_logger().info('Waiting for input odometry...')

    def odom_callback(self, msg: Odometry, qos_id: int):
        self.latest_msg = copy.deepcopy(msg)
        self.received_count += 1

        if self.received_count == 1:
            self.get_logger().info(
                f'Received first odom from {self.input_odom_topic} with qos_id={qos_id}'
            )
            self.get_logger().info(
                f'Original frame: {msg.header.frame_id} -> {msg.child_frame_id}'
            )

        if self.received_count % 100 == 0:
            self.get_logger().info(f'Received odom count: {self.received_count}')

    def timer_callback(self):
        if self.latest_msg is None:
            return

        now = self.get_clock().now().to_msg()

        out = Odometry()
        out.header = copy.deepcopy(self.latest_msg.header)
        out.header.stamp = now
        out.header.frame_id = self.odom_frame_id
        out.child_frame_id = self.base_frame_id

        out.pose = copy.deepcopy(self.latest_msg.pose)
        out.twist = copy.deepcopy(self.latest_msg.twist)

        self.pub.publish(out)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.odom_frame_id
            tf.child_frame_id = self.base_frame_id

            tf.transform.translation.x = out.pose.pose.position.x
            tf.transform.translation.y = out.pose.pose.position.y
            tf.transform.translation.z = out.pose.pose.position.z
            tf.transform.rotation = out.pose.pose.orientation

            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = RobotOdomBridge()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
