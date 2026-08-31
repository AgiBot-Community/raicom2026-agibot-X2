#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class TfPoseBridge(Node):
    def __init__(self):
        super().__init__("tf_pose_bridge")

        self.global_frame = self.declare_parameter("global_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/map_tf_distribution/localization_pose"
        ).value
        self.publish_hz = float(self.declare_parameter("publish_hz", 20.0).value)
        self.lookup_timeout_sec = float(
            self.declare_parameter("lookup_timeout_sec", 0.05).value
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)

        self.timer = self.create_timer(
            1.0 / max(self.publish_hz, 1.0),
            self.timer_callback
        )

        self.get_logger().info(
            f"tf_pose_bridge started: {self.global_frame} -> {self.base_frame}"
        )
        self.get_logger().info(f"Publishing pose to: {self.pose_topic}")

    def timer_callback(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.lookup_timeout_sec),
            )

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.global_frame
            msg.pose.position.x = tf.transform.translation.x
            msg.pose.position.y = tf.transform.translation.y
            msg.pose.position.z = tf.transform.translation.z
            msg.pose.orientation = tf.transform.rotation

            self.pose_pub.publish(msg)

        except TransformException as e:
            self.get_logger().warn(
                f"TF {self.global_frame}->{self.base_frame} not available: {e}",
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = TfPoseBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
