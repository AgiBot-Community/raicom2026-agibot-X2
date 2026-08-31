#!/usr/bin/env python3
from collections import deque
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
import rclpy
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')

TYPE_NAMES = {0: "ARROW", 1: "CUBE", 2: "SPHERE", 3: "CYLINDER", 4: "LINE_STRIP",
              5: "LINE_LIST", 6: "CUBE_LIST", 7: "SPHERE_LIST", 8: "POINTS", 9: "TEXT", 10: "MESH", 11: "TRIANGLE_LIST"}


class ObstaclePolygonEcho(Node):
    def __init__(self):
        super().__init__('obstacle_polygon_echo')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        self.sub = self.create_subscription(
            MarkerArray, "/perception/debug/obstacle_polygon", self.cb, qos)
        self.get_logger().info("Subscribing: /perception/debug/obstacle_polygon")
        self.last_print = self.get_clock().now()
        self.arrivals = deque()

    def cb(self, msg):
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

        if (now - self.last_print).nanoseconds * 1e-9 >= 1.0:
            self.last_print = now
            self._visualize(msg.markers)
            n = len(msg.markers)
            tc = {}
            for m in msg.markers:
                tn = TYPE_NAMES.get(m.type, f"UNK({m.type})")
                tc[tn] = tc.get(tn, 0) + 1
            ts = ", ".join(f"{k}:{v}" for k, v in tc.items())
            cyls = [m for m in msg.markers if m.type == Marker.CYLINDER]
            texts = [m for m in msg.markers if m.type ==
                     Marker.TEXT_VIEW_FACING]
            polys = [m for m in msg.markers if m.type not in (
                Marker.CYLINDER, Marker.ARROW, Marker.TEXT_VIEW_FACING)]
            lines = [f"Obstacle polygon markers\n"
                     f"  num_markers:   {n}\n"
                     f"  num_obstacles: ~{n // 4}\n"
                     f"  types:         {ts}\n"
                     f"  FPS:           {len(self.arrivals):.1f}"]
            for i, c in enumerate(cyls[:5]):
                p = c.pose.position
                txt = texts[i].text if i < len(texts) else ""
                pp = len(polys[i].points) if i < len(polys) else 0
                lines.append(
                    f"  -- [{i}] center=({p.x:.3f},{p.y:.3f},{p.z:.3f}), info=\"{txt}\", poly_pts={pp}")
            if len(cyls) > 5:
                lines.append(f"  ... and {len(cyls) - 5} more")
            self.get_logger().info("\n".join(lines))

    def _visualize(self, markers):
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        for m in markers:
            if m.type == Marker.LINE_STRIP and len(m.points) >= 2:
                pts = np.array([[p.x, p.y, p.z] for p in m.points])
                pts = np.vstack([pts, pts[0:1]])
                ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], 'r-', linewidth=2)
            elif m.type == Marker.CYLINDER:
                ax.scatter([m.pose.position.x], [m.pose.position.y], [m.pose.position.z],
                           c='red', s=50, marker='o', depthshade=False)
            elif m.type == Marker.ARROW:
                p = m.pose.position
                ax.quiver(p.x, p.y, p.z, m.scale.x, m.scale.y, 0,
                          color='yellow', arrow_length_ratio=0.3, linewidth=2)
            elif m.type == Marker.TEXT_VIEW_FACING and m.text:
                ax.text(m.pose.position.x, m.pose.position.y, m.pose.position.z + 0.3,
                        m.text, color='white', fontsize=7,
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.7))

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Obstacle Polygon')
        ax.view_init(elev=50, azim=-60)
        plt.savefig('/tmp/obstacle_polygon.jpg', dpi=120, bbox_inches='tight')
        plt.close(fig)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstaclePolygonEcho()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
