"""Publish a static occupancy map.

The map loading and OccupancyGrid conversion intentionally mirror
the source branch map-loading behavior.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


FREE = 0
OCCUPIED = 100
UNKNOWN = -1


@dataclass(frozen=True)
class GridMap:
    """Occupancy grid in ROS convention.

    `data[i, j]`: i = column (along x), j = row (along y).
    """

    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0
    frame_id: str = "map"

    @property
    def size_x(self) -> int:
        return int(self.data.shape[0])

    @property
    def size_y(self) -> int:
        return int(self.data.shape[1])


def load_map_yaml(yaml_path: str | Path) -> GridMap:
    yaml_path = Path(yaml_path)
    with yaml_path.open("r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    image_rel = meta["image"]
    resolution = float(meta["resolution"])
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    image_path = (yaml_path.parent / image_rel).resolve()
    img = Image.open(image_path).convert("L")
    arr = np.asarray(img, dtype=np.uint8)

    # Pixel (0, 0) of an image is the top-left corner; ROS treats the
    # bottom-left of the image as the map origin.
    arr = np.flipud(arr)
    # Now arr is (rows=y, cols=x). Transpose to (x, y).
    arr = arr.T

    pixel = arr.astype(np.float32)
    if negate == 0:
        p = (255.0 - pixel) / 255.0
    else:
        p = pixel / 255.0

    grid = np.full(arr.shape, UNKNOWN, dtype=np.int8)
    grid[p > occupied_thresh] = OCCUPIED
    grid[p < free_thresh] = FREE

    origin_x = float(origin[0])
    origin_y = float(origin[1])
    if "height" in meta:
        origin_y = origin_y - arr.shape[1] * resolution

    return GridMap(
        data=grid,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=float(origin[2]) if len(origin) > 2 else 0.0,
        frame_id=meta.get("frame_id", "map"),
    )


def to_occupancy_grid_msg(
    gmap: GridMap,
    stamp,
    frame_id: str | None = None,
) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id or gmap.frame_id
    msg.info.resolution = gmap.resolution
    msg.info.width = gmap.size_x
    msg.info.height = gmap.size_y

    pose = Pose()
    pose.position.x = gmap.origin_x
    pose.position.y = gmap.origin_y
    pose.orientation.w = 1.0
    msg.info.origin = pose

    grid_yx = np.ascontiguousarray(gmap.data.T)
    msg.data = grid_yx.flatten().astype(np.int8).tolist()
    return msg


class MapPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("map_publisher_node")

        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period_s", 0.0)

        map_yaml = str(self.get_parameter("map_yaml").value).strip()
        if not map_yaml:
            raise RuntimeError("Parameter 'map_yaml' is required.")
        if not os.path.isabs(map_yaml):
            map_yaml = os.path.abspath(map_yaml)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._gmap = load_map_yaml(map_yaml)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            qos,
        )

        self._timer = None
        self._publish_map()
        period = float(self.get_parameter("publish_period_s").value)
        if period > 0.0:
            self._timer = self.create_timer(period, self._publish_map)

        self.get_logger().info(
            f"Map loaded: {self._gmap.size_x}x{self._gmap.size_y} cells @ "
            f"{self._gmap.resolution} m, origin=({self._gmap.origin_x:.2f},"
            f"{self._gmap.origin_y:.2f}), topic={self.get_parameter('map_topic').value}"
        )

    def _publish_map(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._pub.publish(to_occupancy_grid_msg(self._gmap, stamp, self._frame_id))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
