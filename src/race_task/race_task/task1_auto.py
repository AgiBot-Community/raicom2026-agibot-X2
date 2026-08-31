#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from aimdk_msgs.msg import (
    PncTaskRequest,
    MessageHeader,
    RequestHeader,
    CommonState,
    McActionCommand,
)
from aimdk_msgs.srv import SetMcAction
from geometry_msgs.msg import PoseStamped


TASK_REQUEST_START = 1
TASK_REQUEST_STOP = 2


def yaw_to_quaternion(yaw: float):
    """Convert yaw angle to quaternion. Roll and pitch are zero."""
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


class Task1AutoNode(Node):
    def __init__(self):
        super().__init__("task1_auto")

        # Parameters
        self.declare_parameter("map_id", 1773113429735)
        self.declare_parameter("target_x", 1.0)
        self.declare_parameter("target_y", 2.0)
        self.declare_parameter("target_z", 0.0)
        self.declare_parameter("target_yaw", 0.0)
        self.declare_parameter("target_radius", 0.5)
        self.declare_parameter("pnc_mode", 0)
        self.declare_parameter("max_speed", 0.5)
        self.declare_parameter("set_motion_mode", True)
        self.declare_parameter("stand_mode", "SD")
        self.declare_parameter("locomotion_mode", "LD")
        self.declare_parameter("use_pose_check", True)
        self.declare_parameter("localization_topic", "/map_tf_distribution/localization_pose")
        self.declare_parameter("timeout_sec", 120.0)
        self.declare_parameter("publish_hz", 1.0)

        self.map_id = int(self.get_parameter("map_id").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.target_y = float(self.get_parameter("target_y").value)
        self.target_z = float(self.get_parameter("target_z").value)
        self.target_yaw = float(self.get_parameter("target_yaw").value)
        self.target_radius = float(self.get_parameter("target_radius").value)
        self.pnc_mode = int(self.get_parameter("pnc_mode").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.set_motion_mode = bool(self.get_parameter("set_motion_mode").value)
        self.stand_mode = str(self.get_parameter("stand_mode").value)
        self.locomotion_mode = str(self.get_parameter("locomotion_mode").value)
        self.use_pose_check = bool(self.get_parameter("use_pose_check").value)
        self.localization_topic = str(self.get_parameter("localization_topic").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.task_id = int(time.time() * 1000)
        self.start_time = self.get_clock().now()
        self.current_pose: Optional[PoseStamped] = None
        self.navigation_msg: Optional[PncTaskRequest] = None
        self.finished = False

        self.nav_pub = self.create_publisher(
            PncTaskRequest,
            "/aima/te/pnc_task_request",
            10,
        )

        self.mc_client = self.create_client(
            SetMcAction,
            "/aimdk_5Fmsgs/srv/SetMcAction",
        )

        if self.use_pose_check:
            self.create_subscription(
                PoseStamped,
                self.localization_topic,
                self.pose_callback,
                10,
            )

        self.get_logger().info("Task1 auto node started.")
        self.get_logger().info(
            f"Target: map_id={self.map_id}, "
            f"x={self.target_x:.2f}, y={self.target_y:.2f}, "
            f"yaw={self.target_yaw:.2f}, radius={self.target_radius:.2f}"
        )

        self.run_task()

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def make_header(self) -> MessageHeader:
        header = MessageHeader()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.meas_stamp = now
        header.frame_id = "map"
        header.sequence = 0
        return header

    def make_navigation_start_msg(self) -> PncTaskRequest:
        msg = PncTaskRequest()
        msg.header = self.make_header()
        msg.task_type = 2
        msg.task_request = TASK_REQUEST_START
        msg.pnc_mode = self.pnc_mode
        msg.max_forward_speed = self.max_speed
        msg.task_id = self.task_id
        msg.map_id = self.map_id
        msg.target_pose_radius = self.target_radius
        msg.reserve_info = [0] * 64

        qx, qy, qz, qw = yaw_to_quaternion(self.target_yaw)

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = self.target_x
        target.pose.position.y = self.target_y
        target.pose.position.z = self.target_z
        target.pose.orientation.x = qx
        target.pose.orientation.y = qy
        target.pose.orientation.z = qz
        target.pose.orientation.w = qw

        msg.target_pose = target
        return msg

    def make_navigation_stop_msg(self) -> PncTaskRequest:
        msg = PncTaskRequest()
        msg.header = self.make_header()
        msg.task_type = 2
        msg.task_request = TASK_REQUEST_STOP
        msg.pnc_mode = self.pnc_mode
        msg.max_forward_speed = 0.0
        msg.task_id = self.task_id
        msg.map_id = self.map_id
        msg.target_pose_radius = 0.0
        msg.reserve_info = [0] * 64
        msg.target_pose.pose.orientation.w = 1.0
        return msg

    def motion_abbr_to_action(self, abbr: str) -> str:
        choices = {
            "PD": "PASSIVE_DEFAULT",
            "DD": "DAMPING_DEFAULT",
            "JD": "JOINT_DEFAULT",
            "SD": "STAND_DEFAULT",
            "LD": "LOCOMOTION_DEFAULT",
            "HO": "HEAD_ONLY",
            "US": "UPPERBODY_REMOTE_SPLIT",
        }
        if abbr not in choices:
            raise ValueError(f"Unknown motion abbreviation: {abbr}")
        return choices[abbr]

    def set_mc_action_by_abbr(self, abbr: str) -> bool:
        action_name = self.motion_abbr_to_action(abbr)

        self.get_logger().info(f"Waiting for SetMcAction service to set {action_name}...")
        if not self.mc_client.wait_for_service(timeout_sec=8.0):
            self.get_logger().error("SetMcAction service unavailable.")
            return False

        req = SetMcAction.Request()
        req.header = RequestHeader()
        req.source = "node"

        cmd = McActionCommand()
        cmd.action_desc = action_name
        req.command = cmd

        self.get_logger().info(f"Sending motion mode request: {action_name}")

        for i in range(8):
            req.header.stamp = self.get_clock().now().to_msg()
            future = self.mc_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=0.3)

            if future.done():
                response = future.result()
                if response and response.response.status.value == CommonState.SUCCESS:
                    self.get_logger().info(f"Motion mode set successfully: {action_name}")
                    return True

                if response:
                    self.get_logger().error(
                        f"Failed to set mode {action_name}: {response.response.message}"
                    )
                return False

            self.get_logger().info(f"Retrying motion mode request... [{i}]")

        self.get_logger().error(f"Motion mode request timeout: {action_name}")
        return False

    def distance_to_target(self) -> Optional[float]:
        if self.current_pose is None:
            return None

        dx = self.current_pose.pose.position.x - self.target_x
        dy = self.current_pose.pose.position.y - self.target_y
        return math.sqrt(dx * dx + dy * dy)

    def publish_navigation_start(self):
        if self.navigation_msg is None:
            self.navigation_msg = self.make_navigation_start_msg()

        self.navigation_msg.header = self.make_header()
        self.navigation_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.nav_pub.publish(self.navigation_msg)

    def publish_navigation_stop(self):
        stop_msg = self.make_navigation_stop_msg()
        for _ in range(3):
            stop_msg.header = self.make_header()
            self.nav_pub.publish(stop_msg)
            time.sleep(0.2)

    def run_task(self):
        if self.set_motion_mode:
            self.get_logger().info("Step 1: set stand mode.")
            self.set_mc_action_by_abbr(self.stand_mode)
            time.sleep(2.0)

            self.get_logger().info("Step 2: set locomotion mode.")
            self.set_mc_action_by_abbr(self.locomotion_mode)
            time.sleep(2.0)

        self.get_logger().info("Step 3: start navigation request.")
        self.navigation_msg = self.make_navigation_start_msg()

        period = 1.0 / max(self.publish_hz, 0.1)
        timer = self.create_timer(period, self.loop)

    def loop(self):
        if self.finished:
            return

        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        self.publish_navigation_start()

        dist = self.distance_to_target()
        if dist is not None:
            self.get_logger().info(f"Distance to target: {dist:.2f} m")
            if dist <= self.target_radius:
                self.get_logger().info("Target reached. Sending stop request.")
                self.publish_navigation_stop()
                self.finished = True
                rclpy.shutdown()
                return
        else:
            if self.use_pose_check:
                self.get_logger().warn("No localization pose received yet.")

        if elapsed > self.timeout_sec:
            self.get_logger().warn("Task timeout. Sending stop request.")
            self.publish_navigation_stop()
            self.finished = True
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Task1AutoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Keyboard interrupt. Sending stop request.")
        node.publish_navigation_stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
