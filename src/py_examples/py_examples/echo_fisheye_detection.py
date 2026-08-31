#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from collections import deque


class FisheyeDetectionEcho(Node):
    def __init__(self):
        super().__init__('fisheye_detection_echo')
        self.declare_parameter('dump_video_path', '')
        self.dump_video_path = self.get_parameter('dump_video_path').value
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            Image, "/perception/debug/left_fisheye_detection_viz", self.cb, qos)
        self.get_logger().info("Subscribing: /perception/debug/left_fisheye_detection_viz")
        self.last_print = self.get_clock().now()
        self.arrivals = deque()
        self.bridge = CvBridge()
        self.video_writer = None
        self.frame_count = 0

    def cb(self, msg):
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if (now - self.last_print).nanoseconds * 1e-9 >= 1.0:
            self.last_print = now
            cv2.imwrite('/tmp/fisheye_detection.jpg', cv_img)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"Fisheye detection viz\n"
                f"  frame_id:  {msg.header.frame_id}\n"
                f"  stamp:     {stamp:.6f}\n"
                f"  encoding:  {msg.encoding}\n"
                f"  size:      {msg.width}x{msg.height}\n"
                f"  data size: {len(msg.data)}\n"
                f"  FPS:       {len(self.arrivals):.1f}")
        if self.dump_video_path:
            self._dump(cv_img)

    def _dump(self, cv_img):
        try:
            if self.video_writer is None:
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                h, w = cv_img.shape[:2]
                self.video_writer = cv2.VideoWriter(
                    self.dump_video_path, fourcc, 10.0, (w, h))
            self.video_writer.write(cv_img)
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Video dump error: {e}")

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FisheyeDetectionEcho()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
