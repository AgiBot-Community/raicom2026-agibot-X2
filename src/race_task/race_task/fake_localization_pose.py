#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


def quaternion_from_yaw(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class FakeLocalizationPose(Node):
    def __init__(self):
        super().__init__("fake_localization_pose")

        self.pose_topic = self.declare_parameter(
            "pose_topic", "/map_tf_distribution/localization_pose"
        ).value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value

        self.x = float(self.declare_parameter("initial_x", 0.0).value)
        self.y = float(self.declare_parameter("initial_y", 0.0).value)
        self.yaw = float(self.declare_parameter("initial_yaw", 0.0).value)

        self.publish_hz = float(self.declare_parameter("publish_hz", 20.0).value)
        self.cmd_timeout_sec = float(self.declare_parameter("cmd_timeout_sec", 0.5).value)

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / self.publish_hz, self.loop)

        self.get_logger().info("fake_localization_pose started.")

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9

        if cmd_age <= self.cmd_timeout_sec:
            v = self.last_cmd.linear.x
            w = self.last_cmd.angular.z
        else:
            v = 0.0
            w = 0.0

        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0

        qz, qw = quaternion_from_yaw(self.yaw)
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalizationPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
