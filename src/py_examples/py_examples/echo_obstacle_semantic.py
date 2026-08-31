#!/usr/bin/env python3
from collections import deque
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
import rclpy
from typing import Tuple
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

SEMANTIC_COLORS_BGR = {
    'traversable':     (237, 177, 32),
    'static_obstacle': (126, 47, 142),
    'pedestrian':      (119, 172, 48),
}


def pc2_to_numpy(pc2: PointCloud2) -> Tuple[np.ndarray, np.ndarray]:
    """
    把 ROS2 PointCloud2 转成 numpy 数组
    返回:
        xyz: (N,3) float32
        rgb: (N,3) uint8  (R,G,B)
    """
    data = np.frombuffer(pc2.data, dtype=np.uint8)
    num_points = pc2.width * pc2.height
    data = data.reshape(num_points, pc2.point_step)

    xyz = np.zeros((num_points, 3), dtype=np.float32)
    xyz[:, 0] = data[:, 0:4].view(np.float32).squeeze()
    xyz[:, 1] = data[:, 4:8].view(np.float32).squeeze()
    xyz[:, 2] = data[:, 8:12].view(np.float32).squeeze()

    rgb_packed = data[:, 16:20].view(np.uint32).squeeze()
    rgb = np.zeros((num_points, 3), dtype=np.uint8)
    rgb[:, 0] = (rgb_packed >> 16) & 0xFF   # R
    rgb[:, 1] = (rgb_packed >> 8) & 0xFF   # G
    rgb[:, 2] = rgb_packed & 0xFF   # B

    if not pc2.is_dense:
        valid_mask = ~np.isnan(xyz).any(axis=1)
        xyz = xyz[valid_mask]
        rgb = rgb[valid_mask]

    return xyz, rgb


def classify_by_bgr(rgb):
    """Classify points by semantic BGR color. rgb is (N,3) uint8 (R,G,B)."""
    counts = {}
    for name, (cb, cg, cr) in SEMANTIC_COLORS_BGR.items():
        mask = (rgb[:, 0] == cr) & (rgb[:, 1] == cg) & (rgb[:, 2] == cb)
        counts[name] = int(np.sum(mask))
    counts['unknown'] = len(rgb) - sum(counts.values())
    return counts


class ObstacleSemanticEcho(Node):
    def __init__(self):
        super().__init__('obstacle_semantic_echo')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            PointCloud2, "/perception/debug/obstacle_semantic", self.cb, qos)
        self.get_logger().info("Subscribing: /perception/debug/obstacle_semantic")
        self.last_print = self.get_clock().now()
        self.arrivals = deque()

    def cb(self, msg):
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

        xyz, rgb = pc2_to_numpy(msg)

        if (now - self.last_print).nanoseconds * 1e-9 >= 1.0:
            self.last_print = now
            self._visualize(xyz, rgb)
            num_points = msg.width * msg.height
            counts = classify_by_bgr(rgb)
            sem = ", ".join([f"{k}: {v}" for k, v in counts.items() if v > 0])
            fields = " ".join([f"{f.name}({f.datatype})" for f in msg.fields])
            self.get_logger().info(
                f"Obstacle semantic pointcloud\n"
                f"  frame_id:   {msg.header.frame_id}\n"
                f"  num_points: {num_points}\n"
                f"  fields:     {fields}\n"
                f"  semantics:  {sem}\n"
                f"  FPS:        {len(self.arrivals):.1f}")

    def _visualize(self, xyz, rgb):
        colors = rgb.astype(np.float32) / 255.0

        # Downsample for performance
        if len(xyz) > 3000:
            idx = np.random.choice(len(xyz), 3000, replace=False)
            xyz = xyz[idx]
            colors = colors[idx]

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2],
                   c=colors, s=1, depthshade=False)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Obstacle Semantic Pointcloud')
        ax.view_init(elev=50, azim=-60)
        plt.savefig('/tmp/obstacle_semantic.jpg', dpi=120, bbox_inches='tight')
        plt.close(fig)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleSemanticEcho()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
