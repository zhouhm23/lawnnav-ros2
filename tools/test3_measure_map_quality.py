#!/usr/bin/env python3
"""test3_measure_map_quality.py — DEPRECATED: 地图尺寸精度改为全人工测试，不再使用此脚本。
multi-source map quality measurement."""
import sys, time, math, rclpy
from math import floor, ceil
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from test_utils import yaw_from_quaternion, rotate_360, CSVLogger

# Nav2 costmaps are the primary real-time map sources (RTAB-Map point cloud
# is fused in by Nav2).  /map and /grid_map are fallbacks.
MAP_TOPICS = [
    "/global_costmap/costmap",
    "/local_costmap/costmap",
    "/map",
    "/grid_map",
    "/rtabmap/grid_map",
]

class MeasureMapQuality(Node):
    def __init__(self):
        super().__init__("measure_map_quality")
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._latest_localization = None
        self.create_subscription(PoseWithCovarianceStamped, "/localization_pose", self._localization_callback, 10)
        self._maps = {}
        for t in MAP_TOPICS:
            self._maps[t] = None
            self.create_subscription(OccupancyGrid, t, lambda m, tt=t: self._maps.update({tt: m}), 10)
        self._active_map_topic = None
        self.roi_x_min, self.roi_x_max = 0.0, 1.0
        self.roi_y_min, self.roi_y_max = -1.8, 0.0
        self._csv_logger = None

    def _localization_callback(self, msg): self._latest_localization = msg
    def _get_current_yaw(self):
        if self._latest_localization is None: return None
        p = self._latest_localization.pose.pose
        return yaw_from_quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def run(self):
        self.get_logger().info("Waiting for localization...")
        while self._latest_localization is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        ok = rotate_360(self, self._cmd_vel_pub, self._get_current_yaw, angular_speed=0.5, timeout=30.0)
        if not ok: self.get_logger().warn("Rotation incomplete")
        self._active_map_topic = self._detect_map_source()
        if self._active_map_topic is None:
            self.get_logger().error("No map data on any topic: " + ", ".join(MAP_TOPICS))
            return
        self.get_logger().info(f"Using map source: {self._active_map_topic}")
        self._csv_logger = CSVLogger("/home/ubuntu/ros2_ws/src/tools", "map_quality",
            ["timestamp","elapsed_time","free_cells","occupied_cells","unknown_cells","free_area_sqm","occupied_area_sqm","unknown_area_sqm"])
        self.get_logger().info(f"CSV -> {self._csv_logger.filepath}")
        start = time.time(); last = start; end = start + 60.0
        self.get_logger().info("Starting map quality measurement (auto-stop when stable)...")

        # ── Stability-based early termination ─────────────────────────────
        STABLE_SAMPLES = 5
        same_count = 0
        last_f = last_o = last_u = -1

        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if now - last >= 1.0:
                f, o, u = self._sample(start); last = now
                if f == last_f and o == last_o and u == last_u:
                    same_count += 1
                else:
                    same_count = 0
                last_f, last_o, last_u = f, o, u
                if same_count >= STABLE_SAMPLES:
                    self.get_logger().info(
                        f"Map stable for {STABLE_SAMPLES} samples — stopping early")
                    break

        self._csv_logger.close()
        self.get_logger().info(f"Completed. Data -> {self._csv_logger.filepath}")

    def _detect_map_source(self):
        deadline = time.time() + 5.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            for t in MAP_TOPICS:
                if self._maps[t] is not None:
                    f, o, u = self._count(self._maps[t])
                    if f + o > 0: return t
        for t in MAP_TOPICS:
            if self._maps[t] is not None: return t
        return None

    def _sample(self, start):
        msg = self._maps.get(self._active_map_topic)
        if msg is None: return (-1, -1, -1)
        f, o, u = self._count(msg)
        if f + o + u <= 0: return (-1, -1, -1)
        sq = msg.info.resolution ** 2
        e = time.time() - start
        self._csv_logger.add_row([time.time(), f"{e:.3f}", f, o, u, f"{f*sq:.4f}", f"{o*sq:.4f}", f"{u*sq:.4f}"])
        self.get_logger().info(f"[{e:5.1f}s]  Free:{f*sq:.2f}  Occ:{o*sq:.2f}  Unk:{u*sq:.2f}")
        return (f, o, u)

    def _count(self, msg):
        """Count cells using path_coverage convention:
           -1          = unknown
           0 ..   70   = free / traversable
           otherwise   = obstacle / inflated
        Cell indexing uses floor/ceil like path_coverage_node.py."""
        COSTMAP_MAX_NON_LETHAL = 70
        if msg is None: return (0,0,0)
        w,h,res = msg.info.width, msg.info.height, msg.info.resolution
        ox,oy = msg.info.origin.position.x, msg.info.origin.position.y
        mi = max(0, int(floor((self.roi_x_min - ox)/res)))
        mx = min(w-1, int(ceil((self.roi_x_max - ox)/res)))
        mj = max(0, int(floor((self.roi_y_min - oy)/res)))
        my = min(h-1, int(ceil((self.roi_y_max - oy)/res)))
        if mi > mx or mj > my: return (0,0,0)
        f = o = u = 0
        for j in range(mj, my+1):
            rs = j*w
            for i in range(mi, mx+1):
                v = msg.data[rs+i]
                if v == -1: u += 1
                elif 0 <= v <= COSTMAP_MAX_NON_LETHAL: f += 1
                else: o += 1
        return (f, o, u)

def main(args=None):
    rclpy.init(args=args); node = MeasureMapQuality()
    try: node.run()
    except KeyboardInterrupt: node.get_logger().info("Interrupted - saving data...")
    except Exception as exc: node.get_logger().error(f"Error: {exc}")
    finally:
        try:
            if node._csv_logger is not None: node._csv_logger.close()
        except Exception: pass
        try: node.destroy_node()
        except Exception: pass
        try:
            if rclpy.ok(): rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__": main()
