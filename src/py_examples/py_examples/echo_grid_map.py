#!/usr/bin/env python3
from collections import deque
from grid_map_msgs.msg import GridMap
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
import rclpy
import numpy as np
from matplotlib.colors import ListedColormap
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

SEMANTIC_LABELS = {1: 'traversable', 2: 'static_obstacle', 3: 'pedestrian'}
# RGB colors for matplotlib
SEMANTIC_COLORS_RGB = {
    0: (0.2, 0.2, 0.2),       # unknown - dark gray
    1: (0.1, 0.8, 0.1),       # traversable - green
    2: (0.8, 0.1, 0.1),       # static_obstacle - red
    3: (0.1, 0.4, 0.8),       # pedestrian - blue
}


class GridMapEcho(Node):
    def __init__(self):
        super().__init__('grid_map_echo')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            GridMap, "/perception/grid_map", self.cb, qos)
        self.get_logger().info("Subscribing: /perception/grid_map")
        self.last_print = self.get_clock().now()
        self.arrivals = deque()

    def _parse_layer(self, msg, name):
        if name not in msg.layers:
            return None
        idx = list(msg.layers).index(name)
        mat = msg.data[idx]
        if len(mat.layout.dim) >= 2:
            cols, rows = mat.layout.dim[0].size, mat.layout.dim[1].size
        else:
            return None
        return np.array(mat.data, dtype=np.float32).reshape(cols, rows)

    def cb(self, msg):
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

        sem = self._parse_layer(msg, 'semantic_map')
        elev = self._parse_layer(msg, 'elevation_map')

        if (now - self.last_print).nanoseconds * 1e-9 >= 1.0:
            self.last_print = now
            self._visualize(msg, sem, elev)
            pos = msg.info.pose.position
            layers = ", ".join(msg.layers) if msg.layers else "(none)"
            lines = [
                f"Grid map received\n"
                f"  frame_id:   {msg.header.frame_id}\n"
                f"  resolution: {msg.info.resolution:.4f} m/cell\n"
                f"  size:       {msg.info.length_x:.2f} x {msg.info.length_y:.2f} m\n"
                f"  position:   ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})\n"
                f"  layers:     {layers}\n"
                f"  FPS:        {len(self.arrivals):.1f}"]
            if sem is not None:
                counts = {l: int(np.sum(sem == v))
                          for v, l in SEMANTIC_LABELS.items()}
                lines.append(
                    "  semantic:   " + ", ".join(f"{k}: {v}" for k, v in counts.items() if v > 0))
            if elev is not None:
                valid = elev[np.isfinite(elev)]
                if len(valid) > 0:
                    lines.append(
                        f"  elevation:  min={valid.min():.3f}, max={valid.max():.3f}, mean={valid.mean():.3f}")
            self.get_logger().info("\n".join(lines))

    def _visualize(self, msg, sem, elev):
        if elev is None:
            return
        rows, cols = elev.shape
        res = msg.info.resolution

        # Build grid coordinates
        x = np.arange(cols) * res
        y = np.arange(rows) * res
        x -= x.mean()
        y -= y.mean()
        xx, yy = np.meshgrid(x, y)

        valid = np.isfinite(elev)
        if not np.any(valid):
            return

        xx_v = xx[valid].ravel()
        yy_v = yy[valid].ravel()
        zz_v = elev[valid].ravel()

        # Color by semantic if available, else by elevation
        if sem is not None:
            sem_v = sem[valid].ravel().astype(int)
            colors = np.array([SEMANTIC_COLORS_RGB.get(
                int(s), (0.5, 0.5, 0.5)) for s in sem_v])
        else:
            norm = (zz_v - zz_v.min()) / (zz_v.max() - zz_v.min() + 1e-6)
            colors = plt.cm.jet(norm)[:, :3]

        # Downsample for performance
        if len(xx_v) > 4000:
            idx = np.random.choice(len(xx_v), 4000, replace=False)
            xx_v, yy_v, zz_v, colors = xx_v[idx], yy_v[idx], zz_v[idx], colors[idx]

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')
        ax.bar3d(xx_v, yy_v, np.zeros_like(zz_v), res, res, zz_v,
                 color=colors, alpha=0.9, zsort='average')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Grid Map - Elevation + Semantic')
        ax.view_init(elev=55, azim=-45)
        plt.savefig('/tmp/grid_map.jpg', dpi=120, bbox_inches='tight')
        plt.close(fig)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GridMapEcho()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
