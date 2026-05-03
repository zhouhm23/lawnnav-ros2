#!/usr/bin/env python3
"""
test4_measure_map_accuracy.py — 地图尺寸精度与一致性评估。

物理设置: 规则箱子，左下角 (0.5, 1.2, 0)，对角 (0.76, 1.38, 0.39)。
尺寸: 长(x)=0.26m, 宽(y)=0.18m, 高(z)=0.39m, 体积=0.018252m³。

流程:
  1. 等待 localization
  2. 360° 预旋转（确保 SLAM 充分观测）
  3. 导航 4 目标矩形路径回到原点
  4. 等待地图稳定
  5. 从 2D OccupancyGrid 提取障碍物包络矩形 → 长/宽
  6. 从 3D /rtabmap/cloud_map 点云提取包络六面体 → 长/宽/高
  7. 计算偏差 → 追加写入同一个 CSV 文件（run_id 自增）
"""

import math
import sys
import time
import yaml
import csv
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from nav2_msgs.action import NavigateToPose

import sensor_msgs_py.point_cloud2 as pc2

from test_utils import (
    yaw_from_quaternion,
    make_pose_stamped,
    normalize_angle,
    rotate_360,
    rotate_by_angle,
    AppendingCSVLogger,
)

# ═══════════════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════════════

GOAL_TIMEOUT_SEC = 120.0       # 单目标导航超时
CSV_PATH = "/home/ubuntu/ros2_ws/src/tools/obstacle_dimension_accuracy.csv"
COSTMAP_MAX_NON_LETHAL = 70    # 与 path_coverage / test3 一致（用于地图稳定检测）
COSTMAP_LETHAL = 100           # Nav2 costmap 中 occupied 阈值（>=100 = 真实占据，排除 inflation 膨胀）
RTABMAP_OCC_THRESHOLD = 50     # RTAB-Map 原始 grid_map 障碍阈值（标准值域 0=free, 100=occ）

# 箱子实际参数（世界坐标系，单位 m）
ACTUAL_BOX = {
    "min_x": 0.50, "max_x": 0.76,
    "min_y": 1.20, "max_y": 1.38,
    "min_z": 0.00, "max_z": 0.39,
}
ACTUAL_LENGTH = 0.26   # x
ACTUAL_WIDTH  = 0.18   # y
ACTUAL_HEIGHT = 0.39   # z
ACTUAL_VOLUME = ACTUAL_LENGTH * ACTUAL_WIDTH * ACTUAL_HEIGHT  # 0.018252

# 搜索 ROI（实际位置 ±0.15 m 边距）
# R9 实测：ROI_MARGIN=0.10 时 y 方向盒子顶部缺失（bbox_max_y=1.26 vs 实际 1.38），
# 微扩到 0.15 确保完整捕获，同时仍排除 x≈0.11 墙壁和 y≈0.81 障碍物。
ROI_MARGIN = 0.15
ROI = {
    "x_min": ACTUAL_BOX["min_x"] - ROI_MARGIN,  # 0.35
    "x_max": ACTUAL_BOX["max_x"] + ROI_MARGIN,  # 0.91
    "y_min": ACTUAL_BOX["min_y"] - ROI_MARGIN,  # 1.05
    "y_max": ACTUAL_BOX["max_y"] + ROI_MARGIN,  # 1.53
    # z 在 3D 点云过滤时：盒子在 [0, 0.39]，上下各 0.20m 边距
    # 紧 ROI 可排除天花板/地面杂点
    "z_min": ACTUAL_BOX["min_z"] - 0.20,        # -0.20
    "z_max": ACTUAL_BOX["max_z"] + 0.20,        #  0.59
}

# 8 个实际顶点 (按 v_x,y,z 编码：0=min, 1=max)
ACTUAL_VERTICES = {}
for bx in (0, 1):
    for by in (0, 1):
        for bz in (0, 1):
            key = f"v{bx}{by}{bz}"
            ACTUAL_VERTICES[key] = (
                ACTUAL_BOX["max_x"] if bx else ACTUAL_BOX["min_x"],
                ACTUAL_BOX["max_y"] if by else ACTUAL_BOX["min_y"],
                ACTUAL_BOX["max_z"] if bz else ACTUAL_BOX["min_z"],
            )

# 稳定检测参数
STABLE_SAMPLES = 5     # 连续不变次数
MAX_STABLE_WAIT = 30.0 # 最长等待秒数

# ═══════════════════════════════════════════════════════════════════════════
# MeasureMapAccuracy 节点
# ═══════════════════════════════════════════════════════════════════════════

class MeasureMapAccuracy(Node):
    def __init__(self):
        super().__init__("measure_map_accuracy")

        # ── 发布器 ────────────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Action client ─────────────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── 订阅 ──────────────────────────────────────────────────────────
        self._latest_localization = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self._localization_callback,
            10,
        )

        self._latest_costmap = None
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 self._costmap_callback, 10)

        # RTAB-Map 原始 grid_map（无 Nav2 inflation，值域 0=free/100=occ/-1=unk）
        # 裸名可能是 /grid_map 或 /rtabmap/grid_map，取决于 launch namespace
        self._latest_grid_map = None
        self.create_subscription(OccupancyGrid, "/grid_map",
                                 self._grid_map_callback, 10)
        self.create_subscription(OccupancyGrid, "/rtabmap/grid_map",
                                 self._grid_map_callback, 10)

        # 3D 点云 — TRANSIENT_LOCAL 以获取累计地图
        cloud_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._latest_cloud = None
        self.create_subscription(PointCloud2, "/rtabmap/cloud_map",
                                 self._cloud_callback, qos_profile=cloud_qos)

        # ── 导航参数 ─────────────────────────────────────────────────────
        self.max_v = 0.26
        self.max_w = 1.0
        self._load_velocity_params()

        # ── 4 目标矩形路径（同 test2）────────────────────────────────────
        self._goals = [
            (1.8, 0.0, -math.pi / 2.0),
            (1.8, -1.0, -math.pi),
            (0.0, -1.0, math.pi / 2.0),
            (0.0, 0.0, 0.0),
        ]

        # ── CSV logger（延迟初始化）───────────────────────────────────────
        self._csv_logger = None

        self.get_logger().info(
            "test4: 地图尺寸精度评估已就绪 "
            f"(箱体: {ACTUAL_LENGTH:.2f}×{ACTUAL_WIDTH:.2f}×{ACTUAL_HEIGHT:.2f} m)"
        )

    # ── 参数加载 ─────────────────────────────────────────────────────────

    def _load_velocity_params(self):
        yaml_path = "/home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml"
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                vs = data.get('velocity_smoother', {}).get('ros__parameters', {})
                if 'max_velocity' in vs:
                    max_vels = vs['max_velocity']
                    self.max_v = float(max_vels[0])
                    self.max_w = float(max_vels[2])
        except Exception as e:
            self.get_logger().warn(
                f"无法解析 nav2_params.yaml，使用默认值 v={self.max_v} w={self.max_w}: {e}")

    # ── 回调 ─────────────────────────────────────────────────────────────

    def _localization_callback(self, msg):
        self._latest_localization = msg

    def _costmap_callback(self, msg):
        self._latest_costmap = msg

    def _grid_map_callback(self, msg):
        self._latest_grid_map = msg

    def _cloud_callback(self, msg):
        self._latest_cloud = msg

    # ── 位姿辅助 ─────────────────────────────────────────────────────────

    def _get_current_pose(self):
        if self._latest_localization is None:
            return None
        p = self._latest_localization.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        return (p.position.x, p.position.y, yaw)

    def _get_current_yaw(self):
        if self._latest_localization is None:
            return None
        p = self._latest_localization.pose.pose
        return yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w)

    # ══════════════════════════════════════════════════════════════════════
    # 主流程
    # ══════════════════════════════════════════════════════════════════════

    def run(self):
        self.get_logger().info("等待 Nav2 action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("等待 localization...")
        while self._latest_localization is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # ── 360° 预旋转 ──────────────────────────────────────────────────
        stop_twist = Twist()
        self._cmd_vel_pub.publish(stop_twist)
        time.sleep(0.5)

        ok = rotate_360(self, self._cmd_vel_pub, self._get_current_yaw,
                        angular_speed=0.5, timeout=30.0)
        if not ok:
            self.get_logger().warn("预旋转未完成 — 地图可能有盲区")

        # 等待 Nav2 接管
        self.get_logger().info("等待 2s 让 Nav2 接管 cmd_vel...")
        deadline = time.time() + 2.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        # ── 导航 4 目标（点2到达后逆时针转270度扫视障碍物）──────────────
        # ROTATE_AT_GOAL = {2}  # 仅目标2旋转
        for idx, (x, y, yaw) in enumerate(self._goals, start=1):
            self.get_logger().info(
                f"发送目标 {idx}/{len(self._goals)}: x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
            cp = self._get_current_pose()
            if cp is None:
                self.get_logger().error("定位丢失，无法继续!")
                return
            if not self._send_goal_and_wait(x, y, yaw):
                self.get_logger().error(f"目标 {idx} 失败，终止测试。")
                return

            # if idx in ROTATE_AT_GOAL:
            #     self.get_logger().info(f"目标 {idx} 到达 — 逆时针旋转 270度...")
            #     self._cmd_vel_pub.publish(Twist())
            #     time.sleep(0.5)
            #     rotate_by_angle(self, self._cmd_vel_pub, self._get_current_yaw,
            #                     angle_rad=3*math.pi/2, angular_speed=0.5, timeout=30.0)

        self.get_logger().info("矩形路径完成 ✓ — 等待地图稳定...")

        # ── 等待地图稳定 ─────────────────────────────────────────────────
        map_stable = self._wait_for_map_stable()
        if not map_stable:
            self.get_logger().warn("地图在超时时间内未稳定，使用当前最新数据")

        # ── 提取 2D / 3D 包络盒 ──────────────────────────────────────────
        bbox_2d = self._extract_2d_bbox()
        bbox_3d = self._extract_3d_bbox()

        self.get_logger().info(
            f"2D bbox: length={bbox_2d['length']:.4f} width={bbox_2d['width']:.4f} "
            f"cells={bbox_2d['occ_cells']}")
        self.get_logger().info(
            f"3D bbox: length={bbox_3d['length']:.4f} width={bbox_3d['width']:.4f} "
            f"height={bbox_3d['height']:.4f} points={bbox_3d['point_count']}")

        # ── 计算偏差 ─────────────────────────────────────────────────────
        errors = self._compute_errors(bbox_2d, bbox_3d)

        # ── 写入 CSV ─────────────────────────────────────────────────────
        self._write_csv(bbox_2d, bbox_3d, errors)

        # ── 打印栅格对照表（辅助 RViz 手数格子）─────────────────────────
        self._print_grid_cells_for_manual()

        self.get_logger().info(
            f"✓ test4 完成，数据已写入 {CSV_PATH}"
        )

    # ══════════════════════════════════════════════════════════════════════
    # 导航
    # ══════════════════════════════════════════════════════════════════════

    def _send_goal_and_wait(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("目标被 Nav2 拒绝")
            return False

        result_future = goal_handle.get_result_async()
        start_time = time.time()
        last_localized_time = start_time
        LOCALIZATION_TIMEOUT = 3.0

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            cp = self._get_current_pose()
            if cp is not None:
                last_localized_time = now
            elif now - last_localized_time > LOCALIZATION_TIMEOUT:
                self.get_logger().error(
                    f"定位丢失 {LOCALIZATION_TIMEOUT:.0f}s — 取消目标")
                goal_handle.cancel_goal_async()
                return False

            if now - start_time > GOAL_TIMEOUT_SEC:
                self.get_logger().error(
                    f"目标超时 {GOAL_TIMEOUT_SEC:.0f}s — 取消")
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        return result is not None

    # ══════════════════════════════════════════════════════════════════════
    # 地图稳定等待
    # ══════════════════════════════════════════════════════════════════════

    def _wait_for_map_stable(self):
        """1 Hz 采样 ROI 内 occupied 栅格数，连续 STABLE_SAMPLES 次不变则返回 True."""
        self.get_logger().info("等待地图稳定（检测 ROI 内障碍栅格数）...")
        same_count = 0
        last_occ = -1
        start = time.time()

        while rclpy.ok() and (time.time() - start) < MAX_STABLE_WAIT:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if now - start < 1.0:  # 确保 1 Hz 节奏
                continue

            occ = self._count_occ_in_roi()
            if occ == last_occ and occ >= 0:
                same_count += 1
            else:
                same_count = 0
            last_occ = occ

            elapsed = now - start
            self.get_logger().info(
                f"  [{elapsed:.0f}s] occ_cells={occ} stable_count={same_count}/{STABLE_SAMPLES}")

            if same_count >= STABLE_SAMPLES:
                self.get_logger().info("地图已稳定 ✓")
                return True

            # 重置计时基准
            start = time.time()

        return False

    def _count_occ_in_roi(self):
        """统计 ROI 内障碍栅格数（cost > COSTMAP_MAX_NON_LETHAL）。"""
        msg = self._latest_costmap
        if msg is None:
            return -1
        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        mi = max(0, int((ROI["x_min"] - ox) // res))
        mx = min(w - 1, int((ROI["x_max"] - ox) // res))
        mj = max(0, int((ROI["y_min"] - oy) // res))
        my = min(h - 1, int((ROI["y_max"] - oy) // res))

        if mi > mx or mj > my:
            return 0

        occ = 0
        for j in range(mj, my + 1):
            rs = j * w
            for i in range(mi, mx + 1):
                v = msg.data[rs + i]
                if v > COSTMAP_MAX_NON_LETHAL:
                    occ += 1
        return occ

    # ══════════════════════════════════════════════════════════════════════
    # 2D 包络矩形提取
    # ══════════════════════════════════════════════════════════════════════

    def _extract_2d_bbox(self):
        """提取 ROI 内障碍物的 2D 包络矩形。

        优先使用 RTAB-Map 原始 /grid_map（无 Nav2 inflation 层，值域 0=free/100=occ/-1=unk），
        若不可用则回退到 /global_costmap/costmap。
        """
        result = {
            "length": 0.0, "width": 0.0,
            "min_x": 0.0, "max_x": 0.0,
            "min_y": 0.0, "max_y": 0.0,
            "occ_cells": 0, "valid": False,
            "source": "none",
        }

        # ── 选数据源：优先 RTAB-Map 原始 grid_map ────────────────────────
        msg = self._latest_grid_map
        if msg is not None:
            threshold = RTABMAP_OCC_THRESHOLD   # 50 — 标准 OccupancyGrid
            source_name = "/grid_map (RTAB-Map raw)"
        else:
            msg = self._latest_costmap
            if msg is not None:
                threshold = COSTMAP_LETHAL      # 100 — Nav2 costmap (排除 inflation)
                source_name = "/global_costmap/costmap (Nav2)"
            else:
                self.get_logger().error("_extract_2d_bbox: 无 grid_map 也无 costmap")
                return result

        result["source"] = source_name

        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        mi = max(0, int((ROI["x_min"] - ox) // res))
        mx = min(w - 1, int((ROI["x_max"] - ox) // res))
        mj = max(0, int((ROI["y_min"] - oy) // res))
        my = min(h - 1, int((ROI["y_max"] - oy) // res))

        if mi > mx or mj > my:
            self.get_logger().warn("_extract_2d_bbox: ROI 超出地图范围")
            return result

        # 收集障碍栅格的世界坐标
        xs, ys = [], []
        for j in range(mj, my + 1):
            world_y = oy + (j + 0.5) * res
            rs = j * w
            for i in range(mi, mx + 1):
                v = msg.data[rs + i]
                if v >= threshold:
                    world_x = ox + (i + 0.5) * res
                    xs.append(world_x)
                    ys.append(world_y)

        occ_cells = len(xs)
        result["occ_cells"] = occ_cells

        if occ_cells < 3:
            self.get_logger().warn(
                f"_extract_2d_bbox: {source_name} 障碍栅格不足 ({occ_cells})"
                f" — 阈值={threshold}，尝试降级...")
            # 降级：阈值减半
            fallback_threshold = max(threshold // 2, 1)
            xs, ys = [], []
            for j in range(mj, my + 1):
                world_y = oy + (j + 0.5) * res
                rs = j * w
                for i in range(mi, mx + 1):
                    v = msg.data[rs + i]
                    if v >= fallback_threshold:
                        world_x = ox + (i + 0.5) * res
                        xs.append(world_x)
                        ys.append(world_y)
            occ_cells = len(xs)
            result["occ_cells"] = occ_cells
            if occ_cells < 3:
                self.get_logger().warn(
                    f"_extract_2d_bbox: 降级后仍不足 ({occ_cells}) — ROI 可能不正确")
                return result

        result["min_x"] = min(xs)
        result["max_x"] = max(xs)
        result["min_y"] = min(ys)
        result["max_y"] = max(ys)
        result["length"] = result["max_x"] - result["min_x"]
        result["width"]  = result["max_y"] - result["min_y"]
        result["valid"] = True

        self.get_logger().info(
            f"  2D ({source_name}, threshold>={threshold}): "
            f"{occ_cells} cells → bbox "
            f"x=[{result['min_x']:.3f},{result['max_x']:.3f}] "
            f"y=[{result['min_y']:.3f},{result['max_y']:.3f}] "
            f"L={result['length']:.3f} W={result['width']:.3f}")

        return result

    # ══════════════════════════════════════════════════════════════════════
    # 3D 点云包络六面体提取
    # ══════════════════════════════════════════════════════════════════════

    def _extract_3d_bbox(self):
        """从 /rtabmap/cloud_map PointCloud2 中提取 ROI 内点的 3D 包络六面体。"""
        result = {
            "length": 0.0, "width": 0.0, "height": 0.0,
            "min_x": 0.0, "max_x": 0.0,
            "min_y": 0.0, "max_y": 0.0,
            "min_z": 0.0, "max_z": 0.0,
            "point_count": 0, "valid": False,
        }

        msg = self._latest_cloud
        if msg is None:
            self.get_logger().error("_extract_3d_bbox: 无点云数据")
            return result

        # 检查点云字段
        fields = [f.name for f in msg.fields]
        if "x" not in fields:
            self.get_logger().error("_extract_3d_bbox: 点云缺少 x 字段")
            return result

        xs, ys, zs = [], [], []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            try:
                x, y, z = p[0], p[1], p[2]
            except (IndexError, TypeError):
                continue
            if x is None or y is None or z is None:
                continue
            if not (ROI["x_min"] <= x <= ROI["x_max"]):
                continue
            if not (ROI["y_min"] <= y <= ROI["y_max"]):
                continue
            if not (ROI["z_min"] <= z <= ROI["z_max"]):
                continue
            xs.append(x)
            ys.append(y)
            zs.append(z)

        point_count = len(xs)
        result["point_count"] = point_count

        if point_count < 10:
            self.get_logger().warn(
                f"_extract_3d_bbox: ROI 内点数量不足 ({point_count}) — "
                "3D 箱子可能未被充分观测")
            return result

        # 百分位过滤 x/y/z（5%~95%），排除离散噪点对包络盒的污染
        xs_sorted = sorted(xs)
        ys_sorted = sorted(ys)
        zs_sorted = sorted(zs)
        n = point_count
        p5 = max(0, int(n * 0.05))
        p95 = min(n - 1, int(n * 0.95))
        x_p5, x_p95 = xs_sorted[p5], xs_sorted[p95]
        y_p5, y_p95 = ys_sorted[p5], ys_sorted[p95]
        z_p5, z_p95 = zs_sorted[p5], zs_sorted[p95]

        # 底面修正：盒子贴地放置，但深度相机看不到盒子底面（被自身遮挡）。
        # 若点云 5% 分位 min_z 明显高于地面（>0.05m），则 clamp 到 0.0。
        GROUND_Z = 0.0
        Z_FLOOR_THRESHOLD = 0.05
        min_z_corrected = z_p5
        z_corrected = False
        if z_p5 > GROUND_Z + Z_FLOOR_THRESHOLD:
            min_z_corrected = GROUND_Z
            z_corrected = True
            self.get_logger().warn(
                f"  3D: z_p5={z_p5:.3f} > {Z_FLOOR_THRESHOLD:.3f} — "
                f"底面不可见，clamp 到 {GROUND_Z}（原始 z_p5={z_p5:.3f} 保留在 CSV）")

        result["min_x"] = x_p5
        result["max_x"] = x_p95
        result["min_y"] = y_p5
        result["max_y"] = y_p95
        # CSV 中写百分位 z；高度计算用修正后的 min_z
        result["min_z"] = z_p5
        result["max_z"] = z_p95
        result["length"] = result["max_x"] - result["min_x"]
        result["width"]  = result["max_y"] - result["min_y"]
        result["height"] = z_p95 - min_z_corrected
        result["z_corrected"] = z_corrected
        result["valid"] = True

        self.get_logger().info(
            f"  3D: {point_count} pts → bbox (p5-p95) "
            f"x=[{x_p5:.3f},{x_p95:.3f}] "
            f"y=[{y_p5:.3f},{y_p95:.3f}] "
            f"z=[{z_p5:.3f},{z_p95:.3f}]"
            f"{' (min_z clamped to 0)' if z_corrected else ''}"
            f" L={result['length']:.3f} W={result['width']:.3f} H={result['height']:.3f}")

        return result

    # ══════════════════════════════════════════════════════════════════════
    # 偏差计算
    # ══════════════════════════════════════════════════════════════════════

    def _compute_errors(self, bbox_2d, bbox_3d):
        """计算测量值与实际值之间的各项偏差。

        优先使用 3D 数据计算长、宽、高；
        若 3D 不可用则回退到 2D（高度用 NaN）。
        """
        # ── 确定使用的测量尺寸 ───────────────────────────────────────────
        if bbox_3d["valid"]:
            m_len = bbox_3d["length"]
            m_wid = bbox_3d["width"]
            m_hei = bbox_3d["height"]
            bbox = bbox_3d
        elif bbox_2d["valid"]:
            self.get_logger().warn("3D 不可用，回退到 2D bbox（高度=NaN）")
            m_len = bbox_2d["length"]
            m_wid = bbox_2d["width"]
            m_hei = float("nan")
            bbox = bbox_2d
        else:
            self.get_logger().error("2D 和 3D 均不可用！")
            return {}

        m_vol = m_len * m_wid * m_hei if not (math.isnan(m_hei)) else float("nan")

        # ── 尺寸偏差 ─────────────────────────────────────────────────────
        err_len = m_len - ACTUAL_LENGTH
        err_wid = m_wid - ACTUAL_WIDTH
        err_hei = m_hei - ACTUAL_HEIGHT if not math.isnan(m_hei) else float("nan")
        err_vol = m_vol - ACTUAL_VOLUME if not math.isnan(m_vol) else float("nan")

        err_len_pct = (err_len / ACTUAL_LENGTH * 100.0) if ACTUAL_LENGTH else 0.0
        err_wid_pct = (err_wid / ACTUAL_WIDTH * 100.0) if ACTUAL_WIDTH else 0.0
        err_hei_pct = (err_hei / ACTUAL_HEIGHT * 100.0) if not math.isnan(err_hei) and ACTUAL_HEIGHT else float("nan")
        err_vol_pct = (err_vol / ACTUAL_VOLUME * 100.0) if not math.isnan(err_vol) and ACTUAL_VOLUME else float("nan")

        # ── 顶点误差 ─────────────────────────────────────────────────────
        # 构建测量 bbox 的 8 个角点
        mx0, mx1 = bbox["min_x"], bbox["max_x"]
        my0, my1 = bbox["min_y"], bbox["max_y"]
        # 底面顶点用修正后 z（若做了 clamp），顶面用原始 max_z
        z_bottom = 0.0 if bbox.get("z_corrected") else bbox.get("min_z", 0.0)
        z_top = bbox.get("max_z", ACTUAL_HEIGHT)

        vertex_errors = {}
        for bx in (0, 1):
            for by in (0, 1):
                for bz in (0, 1):
                    key = f"v{bx}{by}{bz}"
                    mx = mx1 if bx else mx0
                    my = my1 if by else my0
                    mz = z_top if bz else z_bottom
                    ax, ay, az = ACTUAL_VERTICES[key]
                    dist = math.sqrt((mx - ax)**2 + (my - ay)**2 + (mz - az)**2)
                    vertex_errors[key] = dist

        valid_vertex_errs = [v for v in vertex_errors.values() if not math.isnan(v)]
        mean_vertex_err = (sum(valid_vertex_errs) / len(valid_vertex_errs)
                           if valid_vertex_errs else float("nan"))
        max_vertex_err = max(valid_vertex_errs) if valid_vertex_errs else float("nan")

        return {
            # 测量值
            "measured_l": m_len, "measured_w": m_wid, "measured_h": m_hei,
            "measured_vol": m_vol,
            # 偏差（绝对值）
            "err_l": err_len, "err_w": err_wid, "err_h": err_hei, "err_vol": err_vol,
            # 偏差（百分比）
            "err_l_pct": err_len_pct, "err_w_pct": err_wid_pct,
            "err_h_pct": err_hei_pct, "err_vol_pct": err_vol_pct,
            # 顶点
            "vertex_errors": vertex_errors,
            "mean_vertex_err": mean_vertex_err,
            "max_vertex_err": max_vertex_err,
        }

    # ══════════════════════════════════════════════════════════════════════
    # CSV 写入 — 三组独立测量 (2D自动 / 3D自动 / 2D手动)
    # ══════════════════════════════════════════════════════════════════════

    def _write_csv(self, bbox_2d, bbox_3d, errors):
        """写入三组独立列: 2D自动(去膨胀), 3D自动(百分位过滤), 2D手动(空)."""
        headers = [
            "timestamp",
            # ── 2D 自动 (已去膨胀层) ────────────────────────────────────
            "2d_auto_l_m", "2d_auto_w_m",
            "2d_auto_min_x", "2d_auto_max_x", "2d_auto_min_y", "2d_auto_max_y",
            "2d_auto_err_l_m", "2d_auto_err_w_m",
            "2d_auto_err_l_pct", "2d_auto_err_w_pct",
            "2d_auto_occ_cells", "2d_auto_source",
            # ── 3D 自动 (百分位过滤 + z clamp) ──────────────────────────
            "3d_auto_l_m", "3d_auto_w_m", "3d_auto_h_m", "3d_auto_vol_m3",
            "3d_auto_min_x", "3d_auto_max_x", "3d_auto_min_y", "3d_auto_max_y",
            "3d_auto_min_z", "3d_auto_max_z",
            "3d_auto_err_l_m", "3d_auto_err_w_m", "3d_auto_err_h_m", "3d_auto_err_vol_m3",
            "3d_auto_err_l_pct", "3d_auto_err_w_pct", "3d_auto_err_h_pct", "3d_auto_err_vol_pct",
            "3d_auto_vertex_err_000", "3d_auto_vertex_err_001",
            "3d_auto_vertex_err_010", "3d_auto_vertex_err_011",
            "3d_auto_vertex_err_100", "3d_auto_vertex_err_101",
            "3d_auto_vertex_err_110", "3d_auto_vertex_err_111",
            "3d_auto_mean_vertex_err", "3d_auto_max_vertex_err",
            "3d_auto_point_count",
            # ── 2D 手动 (RViz 数格子, cell=0.05m, 测试后填入) ──────────
            "manual_cells_l", "manual_cells_w",
            "manual_l_m", "manual_w_m",
            "manual_err_l_m", "manual_err_w_m",
            "manual_err_l_pct", "manual_err_w_pct",
            # ── 参考值 ──────────────────────────────────────────────────
            "actual_l_m", "actual_w_m", "actual_h_m", "actual_vol_m3",
            "notes",
        ]

        self._csv_logger = AppendingCSVLogger(CSV_PATH, headers)
        self.get_logger().info(f"CSV (run_id={self._csv_logger.run_id}) -> {CSV_PATH}")

        # ── 2D 自动 ────────────────────────────────────────────────────
        d2 = bbox_2d
        d2_l = d2.get("length", 0.0)
        d2_w = d2.get("width", 0.0)
        d2_el = d2_l - ACTUAL_LENGTH
        d2_ew = d2_w - ACTUAL_WIDTH
        d2_elp = (d2_el / ACTUAL_LENGTH * 100) if ACTUAL_LENGTH else 0.0
        d2_ewp = (d2_ew / ACTUAL_WIDTH * 100) if ACTUAL_WIDTH else 0.0
        d2_src = d2.get("source", "").split(" ")[0] if d2.get("source") else ""

        # ── 3D 自动 ────────────────────────────────────────────────────
        d3 = bbox_3d
        d3_l = d3.get("length", 0.0)
        d3_w = d3.get("width", 0.0)
        d3_h = d3.get("height", 0.0)
        d3_v = d3_l * d3_w * d3_h if d3.get("valid") else 0.0
        d3_el = d3_l - ACTUAL_LENGTH
        d3_ew = d3_w - ACTUAL_WIDTH
        d3_eh = d3_h - ACTUAL_HEIGHT
        d3_ev = d3_v - ACTUAL_VOLUME
        d3_elp = (d3_el / ACTUAL_LENGTH * 100) if ACTUAL_LENGTH else 0.0
        d3_ewp = (d3_ew / ACTUAL_WIDTH * 100) if ACTUAL_WIDTH else 0.0
        d3_ehp = (d3_eh / ACTUAL_HEIGHT * 100) if ACTUAL_HEIGHT else 0.0
        d3_evp = (d3_ev / ACTUAL_VOLUME * 100) if ACTUAL_VOLUME else 0.0
        ve = errors.get("vertex_errors", {}) if errors else {}
        v8 = [ve.get(f"v{x}{y}{z}", float("nan")) for x in "01" for y in "01" for z in "01"]

        # ── 注释 ───────────────────────────────────────────────────────
        notes_parts = []
        if d2_src:
            notes_parts.append(f"2d_src={d2_src}")
        if d3.get("z_corrected"):
            notes_parts.append("3D_z_clamped")
        if d3.get("point_count", 0) >= 10:
            notes_parts.append("3D_p5p95")
        notes = "; ".join(notes_parts)

        row = [
            f"{time.time():.3f}",
            # 2D auto
            f"{d2_l:.4f}", f"{d2_w:.4f}",
            f"{d2.get('min_x',0):.4f}", f"{d2.get('max_x',0):.4f}",
            f"{d2.get('min_y',0):.4f}", f"{d2.get('max_y',0):.4f}",
            f"{d2_el:.4f}", f"{d2_ew:.4f}",
            f"{d2_elp:.2f}", f"{d2_ewp:.2f}",
            str(d2.get("occ_cells", 0)), d2_src,
            # 3D auto
            f"{d3_l:.4f}", f"{d3_w:.4f}", f"{d3_h:.4f}", f"{d3_v:.6f}",
            f"{d3.get('min_x',0):.4f}", f"{d3.get('max_x',0):.4f}",
            f"{d3.get('min_y',0):.4f}", f"{d3.get('max_y',0):.4f}",
            f"{d3.get('min_z',0):.4f}", f"{d3.get('max_z',0):.4f}",
            f"{d3_el:.4f}", f"{d3_ew:.4f}", f"{d3_eh:.4f}", f"{d3_ev:.6f}",
            f"{d3_elp:.2f}", f"{d3_ewp:.2f}", f"{d3_ehp:.2f}", f"{d3_evp:.2f}",
            *[f"{v:.4f}" for v in v8],
            f"{errors.get('mean_vertex_err', float('nan')):.4f}" if errors else "nan",
            f"{errors.get('max_vertex_err', float('nan')):.4f}" if errors else "nan",
            str(d3.get("point_count", 0)),
            # 2D manual (empty)
            "", "", "", "", "", "", "", "",
            # actual
            f"{ACTUAL_LENGTH:.4f}", f"{ACTUAL_WIDTH:.4f}",
            f"{ACTUAL_HEIGHT:.4f}", f"{ACTUAL_VOLUME:.6f}",
            notes,
        ]
        self._csv_logger.add_row(row)
        self._csv_logger.close()

        self.get_logger().info(
            f"CSV written run_id={self._csv_logger.run_id}: "
            f"2d_auto L={d2_l:.3f} W={d2_w:.3f}  "
            f"3d_auto L={d3_l:.3f} W={d3_w:.3f} H={d3_h:.3f}")

    # ══════════════════════════════════════════════════════════════════════
    # 栅格对照表（辅助 RViz 手数格子）
    # ══════════════════════════════════════════════════════════════════════

    def _print_grid_cells_for_manual(self):
        """打印 ROI 内所有栅格坐标和 cost，辅助 RViz 手数格子。

        Grid cell = 0.05m.  实际箱子: L=0.26m (5-6格), W=0.18m (3-4格).
        """
        msg = self._latest_grid_map or self._latest_costmap
        if msg is None:
            self.get_logger().warn("_print_grid_cells: 无地图数据")
            return

        w, h, res = msg.info.width, msg.info.height, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y

        mi = max(0, int((ROI["x_min"] - ox) // res))
        mx = min(w - 1, int((ROI["x_max"] - ox) // res))
        mj = max(0, int((ROI["y_min"] - oy) // res))
        my = min(h - 1, int((ROI["y_max"] - oy) // res))

        self.get_logger().info("========== 栅格对照表 (cell=0.05m) ==========")
        self.get_logger().info(
            f"ROI: x[{ROI['x_min']:.2f},{ROI['x_max']:.2f}] "
            f"y[{ROI['y_min']:.2f},{ROI['y_max']:.2f}]")
        self.get_logger().info(
            "实际箱: L=0.26m(5-6格) W=0.18m(3-4格) "
            "x[0.50,0.76] y[1.20,1.38]")

        for j in range(my, mj - 1, -1):
            world_y = oy + (j + 0.5) * res
            rs = j * w
            line = f"  y={world_y:.2f} |"
            for i in range(mi, mx + 1):
                v = msg.data[rs + i]
                world_x = ox + (i + 0.5) * res
                tag = "OCC" if v >= 100 else ("INF" if v > 70 else "   ")
                line += f" x={world_x:.2f}[{v:3d}/{tag}]"
            self.get_logger().info(line)

        self.get_logger().info("============================================")


# ═══════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = MeasureMapAccuracy()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断 — 正在关闭...")
    except Exception as exc:
        node.get_logger().error(f"未捕获异常: {exc}")
    finally:
        try:
            if node._csv_logger is not None:
                node._csv_logger.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)

