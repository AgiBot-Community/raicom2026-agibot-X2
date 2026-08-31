#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


class NavState(Enum):
    WAIT_POSE = 0
    WAIT_GOAL = 1
    ALIGN_TO_GOAL = 2
    DRIVE_TO_GOAL = 3
    FINAL_ALIGN = 4
    DONE = 5


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp_abs(value, max_abs):
    return max(-max_abs, min(max_abs, value))


def apply_min_threshold(value, min_abs, max_abs):
    if abs(value) < 1e-6:
        return 0.0

    sign = 1.0 if value > 0.0 else -1.0
    abs_value = min(abs(value), max_abs)

    if abs_value < min_abs:
        abs_value = min_abs

    return sign * abs_value


class ServicePoseNavigator(Node):
    def __init__(self):
        super().__init__("service_pose_nav")

        self.pose_topic = self.declare_parameter(
            "pose_topic", "/map_tf_distribution/localization_pose"
        ).value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.goal_pose_topic = self.declare_parameter(
            "goal_pose_topic", "/race_task/service/goal_pose"
        ).value
        self.nav_status_topic = self.declare_parameter(
            "nav_status_topic", "/race_task/service/nav_status"
        ).value

        self.wait_for_goal_from_rviz = self.declare_parameter(
            "wait_for_goal_from_rviz", True
        ).value

        self.target_x = float(self.declare_parameter("target_x", 0.0).value)
        self.target_y = float(self.declare_parameter("target_y", 0.0).value)
        self.target_yaw = float(self.declare_parameter("target_yaw", 0.0).value)

        self.goal_xy_tolerance = float(
            self.declare_parameter("goal_xy_tolerance", 0.25).value
        )
        self.goal_yaw_tolerance = float(
            self.declare_parameter("goal_yaw_tolerance", 0.17).value
        )
        self.final_yaw_enable = bool(
            self.declare_parameter("final_yaw_enable", True).value
        )

        self.max_forward_velocity = float(
            self.declare_parameter("max_forward_velocity", 0.30).value
        )
        self.min_forward_velocity = float(
            self.declare_parameter("min_forward_velocity", 0.20).value
        )
        self.max_angular_velocity = float(
            self.declare_parameter("max_angular_velocity", 0.35).value
        )
        self.min_angular_velocity = float(
            self.declare_parameter("min_angular_velocity", 0.10).value
        )

        self.k_linear = float(self.declare_parameter("k_linear", 0.45).value)
        self.k_angular = float(self.declare_parameter("k_angular", 0.90).value)
        self.heading_tolerance = float(
            self.declare_parameter("heading_tolerance", 0.25).value
        )

        self.pose_timeout_sec = float(
            self.declare_parameter("pose_timeout_sec", 1.0).value
        )
        self.wait_localization_timeout_sec = float(
            self.declare_parameter("wait_localization_timeout_sec", 60.0).value
        )
        self.control_hz = float(self.declare_parameter("control_hz", 20.0).value)

        self.pose = None
        self.pose_time = None
        self.goal_received = not self.wait_for_goal_from_rviz
        self.reached_published = False

        self.state = NavState.WAIT_POSE
        self.start_time = self.get_clock().now()

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_callback,
            10,
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self.goal_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.nav_status_pub = self.create_publisher(
            String, self.nav_status_topic, 10
        )

        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info("service_pose_nav started.")
        self.get_logger().info(f"Pose topic: {self.pose_topic}")
        self.get_logger().info(f"Cmd topic: {self.cmd_vel_topic}")
        self.get_logger().info(f"Goal topic: {self.goal_pose_topic}")
        self.get_logger().info(f"Nav status topic: {self.nav_status_topic}")

        if self.wait_for_goal_from_rviz:
            self.get_logger().warn(
                f"Waiting for external PoseStamped goal on {self.goal_pose_topic}. "
                "The coordinator will publish the target when service navigation starts."
            )
        else:
            self.get_logger().info(
                f"Using fixed target: x={self.target_x:.3f}, "
                f"y={self.target_y:.3f}, yaw={self.target_yaw:.3f}"
            )

    def pose_callback(self, msg):
        self.pose = msg
        self.pose_time = self.get_clock().now()

    def goal_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != "map":
            self.get_logger().warn(
                f"Goal frame is {msg.header.frame_id}, expected map. "
                "Please set RViz Fixed Frame to map."
            )

        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_yaw = yaw_from_quaternion(msg.pose.orientation)
        self.goal_received = True
        self.reached_published = False

        if self.pose is not None:
            self.state = NavState.ALIGN_TO_GOAL
        else:
            self.state = NavState.WAIT_POSE

        self.get_logger().info(
            f"Received goal: x={self.target_x:.3f}, "
            f"y={self.target_y:.3f}, yaw={self.target_yaw:.3f}"
        )

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def publish_reached_once(self):
        """Publish one completion event for the current service-navigation goal."""
        if self.reached_published:
            return
        self.nav_status_pub.publish(String(data="reached"))
        self.reached_published = True
        self.get_logger().info(
            f"Published navigation status: reached -> {self.nav_status_topic}"
        )

    def get_current_xy_yaw(self):
        p = self.pose.pose.position
        q = self.pose.pose.orientation
        return p.x, p.y, yaw_from_quaternion(q)

    def pose_is_fresh(self):
        if self.pose is None or self.pose_time is None:
            return False
        age = (self.get_clock().now() - self.pose_time).nanoseconds / 1e9
        return age <= self.pose_timeout_sec

    def control_loop(self):
        now = self.get_clock().now()

        if self.pose is None:
            elapsed = (now - self.start_time).nanoseconds / 1e9
            self.publish_stop()

            if elapsed > self.wait_localization_timeout_sec:
                self.get_logger().error(
                    "Localization timeout. No pose received. Stop navigation."
                )
                self.state = NavState.DONE
            else:
                self.state = NavState.WAIT_POSE
            return

        if not self.pose_is_fresh():
            self.publish_stop()
            self.get_logger().warn(
                "Pose timeout. Stop robot until localization recovers.",
                throttle_duration_sec=2.0,
            )
            return

        if not self.goal_received:
            # IMPORTANT: idle navigator must stay silent on /cmd_vel.
            # Both task1_pose_nav and service_pose_nav may be running;
            # publishing zero Twist here would fight the active navigator.
            self.state = NavState.WAIT_GOAL
            return

        x, y, yaw = self.get_current_xy_yaw()

        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.hypot(dx, dy)

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - yaw)
        final_yaw_error = normalize_angle(self.target_yaw - yaw)

        cmd = Twist()

        if self.state in [NavState.WAIT_POSE, NavState.WAIT_GOAL]:
            if distance > self.goal_xy_tolerance:
                self.state = NavState.ALIGN_TO_GOAL
            elif self.final_yaw_enable:
                self.state = NavState.FINAL_ALIGN
            else:
                self.state = NavState.DONE
                self.publish_stop()
                self.publish_reached_once()
                # Return to an idle/silent state so another navigator can use /cmd_vel.
                self.goal_received = False
                self.state = NavState.WAIT_GOAL
                return

        if self.state == NavState.ALIGN_TO_GOAL:
            if distance <= self.goal_xy_tolerance:
                self.state = NavState.FINAL_ALIGN if self.final_yaw_enable else NavState.DONE
            elif abs(heading_error) > self.heading_tolerance:
                raw_w = self.k_angular * heading_error
                cmd.angular.z = apply_min_threshold(
                    raw_w,
                    self.min_angular_velocity,
                    self.max_angular_velocity,
                )
            else:
                self.state = NavState.DRIVE_TO_GOAL

        elif self.state == NavState.DRIVE_TO_GOAL:
            if distance <= self.goal_xy_tolerance:
                self.publish_stop()
                self.state = NavState.FINAL_ALIGN if self.final_yaw_enable else NavState.DONE
                return

            if abs(heading_error) > 0.65:
                self.state = NavState.ALIGN_TO_GOAL
            else:
                raw_v = self.k_linear * distance
                raw_w = self.k_angular * heading_error

                cmd.linear.x = apply_min_threshold(
                    raw_v,
                    self.min_forward_velocity,
                    self.max_forward_velocity,
                )

                cmd.angular.z = clamp_abs(raw_w, self.max_angular_velocity)
                if abs(cmd.angular.z) < self.min_angular_velocity:
                    cmd.angular.z = 0.0

        elif self.state == NavState.FINAL_ALIGN:
            if abs(final_yaw_error) <= self.goal_yaw_tolerance:
                self.publish_stop()
                self.state = NavState.DONE
                self.publish_reached_once()
                self.get_logger().info("Service navigation target reached. Robot stopped.")
                self.goal_received = False
                self.state = NavState.WAIT_GOAL
                return

            raw_w = self.k_angular * final_yaw_error
            cmd.angular.z = apply_min_threshold(
                raw_w,
                self.min_angular_velocity,
                self.max_angular_velocity,
            )

        elif self.state == NavState.DONE:
            # Stop was already sent on the transition to DONE. Stay silent afterwards.
            self.goal_received = False
            self.state = NavState.WAIT_GOAL
            return

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f"state={self.state.name}, "
            f"x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, "
            f"dist={distance:.2f}, head_err={heading_error:.2f}, "
            f"final_err={final_yaw_error:.2f}, "
            f"vx={cmd.linear.x:.2f}, wz={cmd.angular.z:.2f}",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ServicePoseNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                for _ in range(10):
                    node.publish_stop()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
