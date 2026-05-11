#!/usr/bin/env python3

import csv
import math
import os
import time as time_mod
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from std_srvs.srv import Empty

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation


@dataclass
class GridSpec:
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int


def _point_in_polygon_mask(xs: np.ndarray, ys: np.ndarray, polygon_xy: np.ndarray) -> np.ndarray:
    """Vectorized ray-casting point-in-polygon.

    xs, ys: 2D arrays of same shape.
    polygon_xy: (N,2) polygon vertices (not necessarily closed).

    Returns bool mask of same shape.
    """
    if polygon_xy.shape[0] < 3:
        return np.zeros(xs.shape, dtype=bool)

    x = xs
    y = ys

    xv = polygon_xy[:, 0]
    yv = polygon_xy[:, 1]

    # Close polygon
    x0 = xv
    y0 = yv
    x1 = np.roll(xv, -1)
    y1 = np.roll(yv, -1)

    # For each edge, compute intersections with ray to +inf in x.
    # Condition: edge straddles y, and intersection x_int > x.
    # Broadcast edges over grid: (E,1,1) vs (H,W)
    y0e = y0[:, None, None]
    y1e = y1[:, None, None]
    x0e = x0[:, None, None]
    x1e = x1[:, None, None]

    # Avoid division by zero; where y1==y0, edge is horizontal -> no crossing
    dy = (y1e - y0e)
    with np.errstate(divide='ignore', invalid='ignore'):
        x_int = x0e + (y - y0e) * (x1e - x0e) / dy

    cond_straddle = ((y0e > y) != (y1e > y))
    cond_right = x_int > x

    crossings = cond_straddle & cond_right
    inside = np.bitwise_xor.reduce(crossings, axis=0)
    return inside


class CoverageEvaluator(Node):
    def __init__(self):
        super().__init__('coverage_evaluator')

        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('resolution', 0.005)  # meters; < 0.01m as requested
        self.declare_parameter('close_distance', 0.08)  # meters; click near first point to close polygon
        self.declare_parameter('min_polygon_area', 0.05)  # m^2
        self.declare_parameter('coverage_radius', 0.12)  # meters; footprint/coverage half-width
        self.declare_parameter('update_hz', 10.0)
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('log_period_sec', 1.0)
        self.declare_parameter('trajectory_log_dir', '/home/ubuntu/ros2_ws/src/logs/')
        self.declare_parameter('trajectory_log_enabled', True)

        self.clicked_point_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.resolution = float(self.get_parameter('resolution').value)
        self.close_distance = float(self.get_parameter('close_distance').value)
        self.min_polygon_area = float(self.get_parameter('min_polygon_area').value)
        self.coverage_radius = float(self.get_parameter('coverage_radius').value)
        self.update_hz = float(self.get_parameter('update_hz').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)
        self.trajectory_log_dir = self.get_parameter('trajectory_log_dir').get_parameter_value().string_value
        self.trajectory_log_enabled = bool(self.get_parameter('trajectory_log_enabled').value)

        if self.resolution >= 0.01:
            self.get_logger().warn(
                f"resolution={self.resolution:.4f}m is not < 0.01m; "
                "you requested finer than 1cm. Consider 0.005 or 0.002."
            )

        self._clicked_points: List[Tuple[float, float]] = []
        self._polygon_xy: Optional[np.ndarray] = None

        self._grid: Optional[GridSpec] = None
        self._inside_mask: Optional[np.ndarray] = None
        self._covered_mask: Optional[np.ndarray] = None
        self._total_inside: int = 0
        self._covered_inside: int = 0

        # Trajectory log state
        self._traj_csv_file: Optional[object] = None
        self._traj_csv_writer: Optional[object] = None
        self._traj_filename: str = ""

        # Costmap cache
        self._latest_costmap: Optional[OccupancyGrid] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._sub_clicked = self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self._on_clicked_point,
            10,
        )

        # Publish with transient_local so late subscribers get latest ratio (optional but cheap)
        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub_ratio = self.create_publisher(Float32, 'coverage_ratio', pub_qos)

        self._srv_reset = self.create_service(Empty, 'reset', self._on_reset)

        # Subscribe to global costmap (cache latest for later saving)
        self._sub_costmap = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._on_costmap,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )

        update_period = 1.0 / max(self.update_hz, 0.1)
        publish_period = 1.0 / max(self.publish_hz, 0.1)
        self._update_timer = self.create_timer(update_period, self._update_coverage)
        self._publish_timer = self.create_timer(publish_period, self._publish_ratio)

        if self.log_period_sec > 0.0:
            self._log_timer = self.create_timer(self.log_period_sec, self._log_status)
        else:
            self._log_timer = None

        self.get_logger().info(
            f"coverage_evaluator started. Subscribing {self.clicked_point_topic}. "
            f"Frames: {self.global_frame}->{self.base_frame}. res={self.resolution}m."
        )

    def get_ratio(self) -> float:
        if self._total_inside <= 0:
            return 0.0
        return float(self._covered_inside) / float(self._total_inside)

    def log_final(self) -> None:
        if self._grid is None:
            self.get_logger().info('Final coverage: no polygon (ratio=0.0).')
            return
        ratio = self.get_ratio()
        self.get_logger().info(
            f"Final coverage: {ratio * 100.0:.2f}% "
            f"({self._covered_inside}/{self._total_inside} cells)."
        )

    def _on_reset(self, _req: Empty.Request, _res: Empty.Response) -> Empty.Response:
        self.get_logger().info('Reset requested; clearing polygon and coverage state.')
        self._close_trajectory_log()
        self._clicked_points = []
        self._polygon_xy = None
        self._grid = None
        self._inside_mask = None
        self._covered_mask = None
        self._total_inside = 0
        self._covered_inside = 0
        return _res

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        """Cache the latest global costmap for saving when polygon is finalized."""
        self._latest_costmap = msg

    def _on_clicked_point(self, msg: PointStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f"clicked point frame_id='{msg.header.frame_id}' != global_frame='{self.global_frame}'. "
                "Assuming coordinates are already in global_frame."
            )

        x = float(msg.point.x)
        y = float(msg.point.y)

        if self._polygon_xy is not None:
            # Polygon already finalized; ignore further clicks until reset.
            return

        if len(self._clicked_points) == 0:
            self._clicked_points.append((x, y))
            self.get_logger().info(f"First vertex set: ({x:.3f}, {y:.3f})")
            return

        # If close to first point and we already have >=3 vertices, close polygon
        x0, y0 = self._clicked_points[0]
        if len(self._clicked_points) >= 3:
            if math.hypot(x - x0, y - y0) <= self.close_distance:
                self.get_logger().info('Polygon closed; building grid/masks...')
                self._finalize_polygon()
                return

        # Debounce near-duplicate points
        xl, yl = self._clicked_points[-1]
        if math.hypot(x - xl, y - yl) < 0.005:
            return

        self._clicked_points.append((x, y))
        self.get_logger().info(f"Vertex added ({len(self._clicked_points)}): ({x:.3f}, {y:.3f})")

    def _finalize_polygon(self) -> None:
        pts = np.asarray(self._clicked_points, dtype=np.float64)
        if pts.shape[0] < 3:
            self.get_logger().warn('Not enough points to form a polygon.')
            return

        # Compute polygon signed area (shoelace)
        x = pts[:, 0]
        y = pts[:, 1]
        area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
        if area < self.min_polygon_area:
            self.get_logger().warn(f"Polygon area too small: {area:.4f} m^2; ignoring. Use reset and re-click.")
            return

        # Bounding box -> grid
        min_x = float(np.min(pts[:, 0]))
        max_x = float(np.max(pts[:, 0]))
        min_y = float(np.min(pts[:, 1]))
        max_y = float(np.max(pts[:, 1]))

        # Add padding equal to coverage radius so circle updates don't run out-of-bounds at edges
        pad = max(self.coverage_radius, self.resolution)
        min_x -= pad
        min_y -= pad
        max_x += pad
        max_y += pad

        width = int(math.ceil((max_x - min_x) / self.resolution))
        height = int(math.ceil((max_y - min_y) / self.resolution))

        # Guard rails: your 2x2m area is fine, but avoid accidental huge grids
        if width * height > 5_000_000:
            self.get_logger().error(
                f"Grid too large: {width}x{height}={width*height}. "
                "Reduce area or increase resolution."
            )
            return

        self._grid = GridSpec(origin_x=min_x, origin_y=min_y, resolution=self.resolution, width=width, height=height)
        self._polygon_xy = pts

        # Build inside_mask using vectorized point-in-polygon on cell centers
        xs = min_x + (np.arange(width, dtype=np.float64) + 0.5) * self.resolution
        ys = min_y + (np.arange(height, dtype=np.float64) + 0.5) * self.resolution
        X, Y = np.meshgrid(xs, ys)

        inside = _point_in_polygon_mask(X, Y, pts)
        self._inside_mask = inside
        self._covered_mask = np.zeros_like(inside, dtype=bool)

        self._total_inside = int(np.count_nonzero(inside))
        self._covered_inside = 0

        # ── Open trajectory log CSV ──────────────────────────────────
        self._open_trajectory_log()

        # ── Save cached costmap ──────────────────────────────────────
        self._save_costmap()

        self.get_logger().info(
            f"Polygon accepted (area={area:.3f} m^2). Grid={width}x{height} res={self.resolution}m. "
            f"Inside cells={self._total_inside}."
        )

    def _lookup_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Look up robot pose (x, y, yaw) in global frame via TF."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        q = tf.transform.rotation
        r = Rotation.from_quat([q.x, q.y, q.z, q.w])
        _, _, yaw = r.as_euler('xyz', degrees=False)
        return x, y, yaw

    def _update_coverage(self) -> None:
        if self._grid is None or self._inside_mask is None or self._covered_mask is None:
            return

        pose = self._lookup_robot_pose()
        if pose is None:
            return

        rx, ry, yaw = pose
        g = self._grid

        # ── Write trajectory CSV ─────────────────────────────────────
        self._write_trajectory_row(rx, ry, yaw)

        # Convert to grid indices
        cx = int((rx - g.origin_x) / g.resolution)
        cy = int((ry - g.origin_y) / g.resolution)

        rad_cells = int(math.ceil(self.coverage_radius / g.resolution))
        x0 = max(cx - rad_cells, 0)
        x1 = min(cx + rad_cells + 1, g.width)
        y0 = max(cy - rad_cells, 0)
        y1 = min(cy + rad_cells + 1, g.height)

        if x0 >= x1 or y0 >= y1:
            return

        # Compute circle mask in this window
        xs = g.origin_x + (np.arange(x0, x1, dtype=np.float64) + 0.5) * g.resolution
        ys = g.origin_y + (np.arange(y0, y1, dtype=np.float64) + 0.5) * g.resolution
        X, Y = np.meshgrid(xs, ys)
        circle = (X - rx) ** 2 + (Y - ry) ** 2 <= (self.coverage_radius ** 2)

        inside_w = self._inside_mask[y0:y1, x0:x1]
        covered_w = self._covered_mask[y0:y1, x0:x1]

        new_cov = circle & inside_w
        delta = new_cov & (~covered_w)
        if np.any(delta):
            self._covered_inside += int(np.count_nonzero(delta))
            covered_w |= new_cov
            self._covered_mask[y0:y1, x0:x1] = covered_w

    def _publish_ratio(self) -> None:
        msg = Float32()
        msg.data = float(self.get_ratio())
        self._pub_ratio.publish(msg)

    def _log_status(self) -> None:
        if self._polygon_xy is None or self._grid is None or self._inside_mask is None:
            self.get_logger().info(f"Waiting polygon... clicked_points={len(self._clicked_points)}")
            return

        # If TF is missing, ratio will not change; still report current value.
        ratio = self.get_ratio()
        self.get_logger().info(
            f"Coverage: {ratio * 100.0:.2f}% ({self._covered_inside}/{self._total_inside} cells)"
        )

    # ── Trajectory + costmap logging ─────────────────────────────────

    def _open_trajectory_log(self) -> None:
        """Open a timestamped CSV file for trajectory logging."""
        if not self.trajectory_log_enabled:
            return
        try:
            os.makedirs(self.trajectory_log_dir, exist_ok=True)
            ts = time_mod.strftime('%Y%m%d_%H%M%S')
            self._traj_filename = os.path.join(
                self.trajectory_log_dir, f'trajectory_{ts}.csv')
            self._traj_csv_file = open(self._traj_filename, 'w', newline='')
            self._traj_csv_writer = csv.writer(self._traj_csv_file)
            self._traj_csv_writer.writerow(['t', 'x', 'y', 'yaw'])
            self.get_logger().info(f'Trajectory log opened: {self._traj_filename}')
        except Exception as e:
            self.get_logger().warn(f'Failed to open trajectory log: {e}')

    def _write_trajectory_row(self, x: float, y: float, yaw: float) -> None:
        """Append one row to the trajectory CSV."""
        if self._traj_csv_writer is None:
            return
        try:
            t = self.get_clock().now().nanoseconds / 1e9
            self._traj_csv_writer.writerow([f'{t:.6f}', f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
        except Exception:
            pass  # Silently ignore write errors during coverage

    def _close_trajectory_log(self) -> None:
        """Close the trajectory CSV file if open."""
        if self._traj_csv_file is not None:
            try:
                self._traj_csv_file.close()
                self.get_logger().info(
                    f'Trajectory log closed: {self._traj_filename}')
            except Exception:
                pass
            self._traj_csv_file = None
            self._traj_csv_writer = None

    def _save_costmap(self) -> None:
        """Save the cached global costmap as NPZ for offline plotting."""
        if self._latest_costmap is None:
            self.get_logger().warn('No costmap cached; skipping costmap save.')
            return
        try:
            cm = self._latest_costmap
            data = np.array(cm.data, dtype=np.int8).reshape(
                cm.info.height, cm.info.width)
            ts = time_mod.strftime('%Y%m%d_%H%M%S')
            costmap_path = os.path.join(
                self.trajectory_log_dir, f'costmap_{ts}.npz')
            np.savez_compressed(
                costmap_path,
                data=data,
                origin_x=cm.info.origin.position.x,
                origin_y=cm.info.origin.position.y,
                resolution=cm.info.resolution,
                width=cm.info.width,
                height=cm.info.height,
            )
            self.get_logger().info(f'Costmap saved: {costmap_path}')
        except Exception as e:
            self.get_logger().warn(f'Failed to save costmap: {e}')


def main() -> None:
    rclpy.init()
    node = CoverageEvaluator()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.log_final()
            node._close_trajectory_log()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
