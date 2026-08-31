"""Read localization from TF and republish it as PoseStamped.

This mirrors the source branch TF lookup contract: lookup
`global_frame <- base_frame` with a short timeout and warn throttled when
localization is unavailable.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalizationTfNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_tf_node")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_topic", "/map_tf_distribution/localization_pose")
        self.declare_parameter("pose_rate_hz", 20.0)
        self.declare_parameter("tf_lookup_timeout_s", 0.05)

        self._global_frame = str(self.get_parameter("global_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tf_timeout_s = float(
            self.get_parameter("tf_lookup_timeout_s").value)

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pose_pub = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("pose_topic").value),
            10,
        )

        period = 1.0 / max(1e-6, float(self.get_parameter("pose_rate_hz").value))
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"Reading localization TF {self._global_frame}<-{self._base_frame} "
            f"and publishing {self.get_parameter('pose_topic').value}")

    def _lookup_pose(self) -> Optional[Tuple[float, float, float, object]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._tf_timeout_s),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF {self._global_frame}<-{self._base_frame} not available: {e}",
                throttle_duration_sec=2.0,
            )
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return t.x, t.y, _yaw_from_quat(r.x, r.y, r.z, r.w), tf

    def _on_timer(self) -> None:
        pose = self._lookup_pose()
        if pose is None:
            return
        _, _, _, tf = pose
        msg = PoseStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self._global_frame
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        msg.pose.orientation = tf.transform.rotation
        self._pose_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
