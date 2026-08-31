#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class GoalCapture(Node):
    def __init__(self):
        super().__init__("goal_capture")
        self.first_point = None

        self.create_subscription(PointStamped, "/clicked_point", self.clicked_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_cb, 10)

        self.get_logger().info("goal_capture started.")
        self.get_logger().info("Use RViz Publish Point: first click target circle, second click facing point.")
        self.get_logger().info("Or use RViz 2D Goal Pose to get x, y, yaw directly.")

    def clicked_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != "map":
            self.get_logger().warn(f"clicked_point frame is {msg.header.frame_id}, expected map.")

        x = msg.point.x
        y = msg.point.y

        if self.first_point is None:
            self.first_point = (x, y)
            self.get_logger().info(f"Target point saved: x={x:.3f}, y={y:.3f}")
            self.get_logger().info("Now click a second point in the direction the robot should face.")
        else:
            x1, y1 = self.first_point
            yaw = math.atan2(y - y1, x - x1)
            self.get_logger().info("\nCopy these parameters into task1_app_ready.yaml if needed:\n")
            print("target_x: %.3f" % x1)
            print("target_y: %.3f" % y1)
            print("target_yaw: %.3f" % yaw)
            print("")
            self.first_point = None

    def goal_pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)

        self.get_logger().info("\nRViz 2D Goal Pose received. Parameters:\n")
        print("target_x: %.3f" % x)
        print("target_y: %.3f" % y)
        print("target_yaw: %.3f" % yaw)
        print("")


def main(args=None):
    rclpy.init(args=args)
    node = GoalCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
