#!/usr/bin/env python3
"""
test2_nav_cte_and_obstacle_test.py — 导航控制与避障指标测试

用法:
    python3 test2_nav_cte_and_obstacle_test.py --mode cte                     # 仅直线跟踪CTE
    python3 test2_nav_cte_and_obstacle_test.py --mode obstacle --path 1to3    # 仅避障 1→3
    python3 test2_nav_cte_and_obstacle_test.py --mode obstacle --path 4to2    # 仅避障 4→2
    python3 test2_nav_cte_and_obstacle_test.py --mode all                     # 全部（默认，仅CTE）

> 避障模式每次只测一条路径（1→3 或 4→2），避免累计里程计漂移。
> 需要测另一条路径时，Ctrl+C 后重新启动脚本指定 --path。

测试路径（map 坐标系，x⁺=车头，y⁺=左侧）:
  CTE 闭合矩形:
    1→2: 起点 (0, 0) → 终点 (1.8, 0),   期望 y≡0,   1.8m 水平
    2→3: 起点 (1.8, 0) → 终点 (1.8, -1.0), 期望 x≡1.8, 1.0m 竖直
    3→4: 起点 (1.8, -1.0) → 终点 (0, -1.0),  期望 y≡-1.0, 1.8m 水平
    4→1: 起点 (0, -1.0) → 终点 (0, 0),     期望 x≡0,   1.0m 竖直
    1→3: 起点 (0, 0) → 终点 (1.8, -1.0)
    4→2: 起点 (0, -1.0) → 终点 (1.8, 0)

指标:
  --mode cte:
    - 沿闭合矩形 1→2→3→4→1 逐段导航，1Hz 采样轨迹
    - 逐段 CTE RMSE / Max CTE + 整体汇总
    - 轨迹日志 trajectory_cte_{label}_{timestamp}.csv

  --mode obstacle:
    - 碰撞次数：到达目标后人工终端输入 (0/1)
    - 最小轮廓距离间隙 d_min (mm)：由 SLAM 轨迹 + 障碍物几何自动计算
    - 目标到达精度：位置残差 & 航向残差
    - 结果写入 obstacle_avoidance_results.csv
    - 轨迹写入 trajectory_obs_<timestamp>.csv
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

from test_utils import (
    yaw_from_quaternion,
    make_pose_stamped,
    normalize_angle,
    CSVLogger,
    AppendingCSVLogger,
)

# ═══════════════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════════════

GOAL_TIMEOUT_SEC = 120.0
LOCALIZATION_LOST_TIMEOUT = 3.0
CTE_SAMPLE_RATE_HZ = 1.0          # CTE 采样频率

OBSTACLE_RUNS = 3                 # 每条避障路径重复次数

POSE_TOPIC_CANDIDATES = [
    "/rtabmap/localization_pose",
    "/localization_pose",
]

# CTE 闭合矩形路径（map 坐标系，与 test1 GOALS_MAP 一致）
# (label, goal_pose, axis, ref_value)
# axis="y": 水平直线 y≡ref → e_cte = |p_y - ref|
# axis="x": 竖直直线 x≡ref → e_cte = |p_x - ref|
CTE_LOOP_SEGMENTS = [
    ("1→2", (1.8,  0.0, -math.pi / 2.0), "y", 0.0),      # 水平 y≡0,   1.8m
    ("2→3", (1.8, -1.0, -math.pi),       "x", 1.8),       # 竖直 x≡1.8, 1.0m
    ("3→4", (0.0, -1.0,  math.pi / 2.0), "y", -1.0),      # 水平 y≡-1.0, 1.8m
    ("4→1", (0.0,  0.0,  0.0),           "x", 0.0),       # 竖直 x≡0,   1.0m
]
CTE_START_POINT = (0.0, 0.0, 0.0)  # 点1，每 run 先导航到这里

# 避障测试路径（map 坐标系）
OBSTACLE_PATHS = [
    # (label, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)
    ("1→3", 0.0,  0.0,  0.0,                 1.8, -1.0, -math.pi),
    ("4→2", 0.0, -1.0,  math.pi / 2.0,       1.8,  0.0, -math.pi / 2.0),
]

OBS_SAMPLE_RATE_HZ = 1.0          # 避障轨迹采样频率

# ── 障碍物箱（map 坐标系，与 test1/test3/test4 完全一致）──────────────
# 论文坐标: 左下角 (0.9, 1.2), 长 0.26m(x⁺), 宽 0.18m(y⁺)
# 转换 (x_map=y_paper, y_map=-x_paper+0.4):
#   左下角 map: (1.2, -0.5), 右上角 map: (1.38, -0.76)
OBSTACLE_BOX = {
    "x_min": 1.2,  "x_max": 1.38,   # span 0.18m (map-x = paper-y)
    "y_min": -0.76, "y_max": -0.5,   # span 0.26m (map-y = -paper-x+0.4)
}

# ── 车体包络矩形（长边沿车头方向 = +x）──────────────────────────────────
CAR_LENGTH = 0.215   # 沿 x（车头方向）的全长
CAR_WIDTH  = 0.18    # 沿 y（车体左侧方向）的全宽
CAR_HALF_L = CAR_LENGTH / 2.0  # 0.1075
CAR_HALF_W = CAR_WIDTH  / 2.0  # 0.09

# CSV 输出路径
CSV_CTE_PATH = "/home/ubuntu/ros2_ws/src/tools/cte_results.csv"
CSV_OBSTACLE_PATH = "/home/ubuntu/ros2_ws/src/tools/obstacle_avoidance_results.csv"
TRAJECTORY_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs"


# ═══════════════════════════════════════════════════════════════════════════
# NavCteObstacleTest 节点
# ═══════════════════════════════════════════════════════════════════════════

class NavCteObstacleTest(Node):
    def __init__(self, mode: str = "all", obstacle_path: str = ""):
        super().__init__("nav_cte_obstacle_test")
        self._mode = mode
        self._obstacle_path = obstacle_path  # "1to3" or "4to2"
        self._do_cte = mode in ("cte", "all")
        self._do_obstacle = (mode in ("obstacle", "all") and obstacle_path != "")

        # ── 多源定位订阅 ────────────────────────────────────────────────
        self._pose_cache = {}
        self._active_pose_topic = None
        for topic in POSE_TOPIC_CANDIDATES:
            self.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                self._make_pose_callback(topic),
                10,
            )
            self._pose_cache[topic] = None

        # ── Action client ────────────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── 当前活跃的 goal_handle（用于 Ctrl+C 取消）─────────────────────
        self._active_goal_handle = None

    # ── 回调 ─────────────────────────────────────────────────────────────

    def _make_pose_callback(self, topic: str):
        def cb(msg):
            self._pose_cache[topic] = msg
            if self._active_pose_topic is None and msg is not None:
                self._active_pose_topic = topic
                self.get_logger().info(f"定位源已激活: {topic}")
        return cb

    # ── 位姿辅助 ─────────────────────────────────────────────────────────

    def _get_latest_pose_msg(self):
        if self._active_pose_topic is None:
            return None
        return self._pose_cache.get(self._active_pose_topic)

    def _get_current_pose(self):
        msg = self._get_latest_pose_msg()
        if msg is None:
            return None
        p = msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        return (p.position.x, p.position.y, yaw)

    # ══════════════════════════════════════════════════════════════════════
    # 主入口
    # ══════════════════════════════════════════════════════════════════════

    def run(self):
        try:
            self.get_logger().info(f"模式: {self._mode}")

            # 等待多源 localization
            self.get_logger().info(
                f"等待定位数据 (候选 topics: {', '.join(POSE_TOPIC_CANDIDATES)})...")
            deadline = time.time() + 15.0
            while self._active_pose_topic is None and rclpy.ok() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if self._active_pose_topic is None:
                self.get_logger().error(
                    f"{15}s 内无任何定位数据！检查 topics: {POSE_TOPIC_CANDIDATES}")
                return

            self.get_logger().info(f"使用定位源: {self._active_pose_topic}")
            cp = self._get_current_pose()
            if cp is not None:
                self.get_logger().info(
                    f"初始位姿 | map: ({cp[0]:.3f}, {cp[1]:.3f}, {math.degrees(cp[2]):.1f}°)"
                )

            # 等待 Nav2
            self.get_logger().info("等待 Nav2 action server...")
            self._action_client.wait_for_server()

            # ── 执行 CTE 测试 ────────────────────────────────────────────
            if self._do_cte:
                self._run_cte_test()

            # ── 执行避障测试 ────────────────────────────────────────────
            if self._do_obstacle:
                self._run_obstacle_test()

            self.get_logger().info("所有测试完成 ✓")

        except KeyboardInterrupt:
            self.get_logger().info("收到 Ctrl+C — 取消当前导航目标...")
            if self._active_goal_handle is not None:
                self._active_goal_handle.cancel_goal_async()
            self.destroy_node()

    # ══════════════════════════════════════════════════════════════════════
    # CTE 测试
    # ══════════════════════════════════════════════════════════════════════

    def _run_cte_test(self):
        self.get_logger().info("========== CTE 闭合矩形跟踪测试开始 ==========")

        # Step 1: 导航到点1（起点）
        sx, sy, syaw = CTE_START_POINT
        self.get_logger().info(
            f"--- 导航到起点 1 --- map: ({sx:.2f}, {sy:.2f}, "
            f"{math.degrees(syaw):.0f}°)")
        if not self._send_goal_and_wait(sx, sy, syaw):
            self.get_logger().error("无法到达起点1，CTE 测试中止。")
            return
        self.get_logger().info("已到达起点1 ✓")

        # Step 2: 逐段导航并采样轨迹
        seg_results = []    # (label, traj, cte_list, rmse, max_cte, n, dur)
        all_cte = []        # 所有段的 CTE 合并

        for label, (gx, gy, gyaw), axis, ref_val in CTE_LOOP_SEGMENTS:
            self.get_logger().info(
                f"\n--- 段 {label}: 期望 {'y' if axis=='y' else 'x'}≡{ref_val} ---")

            trajectory, duration_s = self._send_goal_and_log_trajectory(
                gx, gy, gyaw, sample_rate_hz=CTE_SAMPLE_RATE_HZ
            )
            if trajectory is None:
                self.get_logger().error(f"段 {label} 导航失败，CTE 测试中止。")
                return

            # ── 计算 CTE ────────────────────────────────────────────────
            if axis == "y":
                cte_values = [abs(py - ref_val) for (_, _, py, _) in trajectory]
            else:  # axis == "x"
                cte_values = [abs(px - ref_val) for (_, px, _, _) in trajectory]

            K = len(cte_values)
            rmse = math.sqrt(sum(e ** 2 for e in cte_values) / K) if K else 0.0
            max_cte = max(cte_values) if cte_values else 0.0

            seg_results.append((label, trajectory, cte_values, rmse, max_cte, K, duration_s))
            all_cte.extend(cte_values)

            self.get_logger().info(
                f"  段 {label}: {K} 点, {duration_s:.1f}s, "
                f"RMSE={rmse:.4f}m, Max={max_cte:.4f}m")

            # 写入段轨迹 CSV
            self._write_trajectory_csv(trajectory, cte_values, label)

        # ── 整体汇总 ────────────────────────────────────────────────────
        all_N = len(all_cte)
        all_rmse = math.sqrt(sum(e ** 2 for e in all_cte) / all_N) if all_N else 0.0
        all_max = max(all_cte) if all_cte else 0.0
        total_dur = sum(r[6] for r in seg_results)

        self.get_logger().info("\n" + "=" * 70)
        self.get_logger().info("             闭合矩形 CTE 测试结果")
        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f"  {'段':>6} {'点数':>5} {'耗时(s)':>8} {'RMSE(m)':>10} {'Max(m)':>10}")
        self.get_logger().info("-" * 70)
        for label, _, _, rmse, max_cte, K, dur in seg_results:
            self.get_logger().info(
                f"  {label:>6} {K:>5} {dur:>8.1f} {rmse:>10.4f} {max_cte:>10.4f}")
        self.get_logger().info("-" * 70)
        self.get_logger().info(
            f"  {'整体':>6} {all_N:>5} {total_dur:>8.1f} {all_rmse:>10.4f} {all_max:>10.4f}")
        self.get_logger().info("=" * 70 + "\n")

        # ── 写入汇总 CSV ────────────────────────────────────────────────
        self._write_cte_csv(seg_results, all_rmse, all_max, all_N, total_dur)

        self.get_logger().info("CTE 测试完成 ✓")

    # ══════════════════════════════════════════════════════════════════════
    # 避障测试
    # ══════════════════════════════════════════════════════════════════════

    def _run_obstacle_test(self):
        # ── 选择本次测试的路径 ──────────────────────────────────────
        path_key = self._obstacle_path  # "1to3" or "4to2"
        if path_key == "1to3":
            label, sx, sy, syaw, gx, gy, gyaw = OBSTACLE_PATHS[0]
        elif path_key == "4to2":
            label, sx, sy, syaw, gx, gy, gyaw = OBSTACLE_PATHS[1]
        else:
            self.get_logger().error(f"未知路径: {path_key}，使用 --path 1to3|4to2")
            return

        self.get_logger().info("========== 避障测试开始 ==========")
        self.get_logger().info(f"本次路径: {label}")

        # 打印障碍物和车体几何信息
        self.get_logger().info(
            f"障碍物箱(map): x∈[{OBSTACLE_BOX['x_min']:.2f},{OBSTACLE_BOX['x_max']:.2f}] "
            f"y∈[{OBSTACLE_BOX['y_min']:.2f},{OBSTACLE_BOX['y_max']:.2f}]")
        self.get_logger().info(
            f"车体包络: {CAR_LENGTH:.3f}m×{CAR_WIDTH:.3f}m "
            f"(半长={CAR_HALF_L:.4f}, 半宽={CAR_HALF_W:.4f})")

        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  路径 {label}: ({sx:.1f},{sy:.1f}) → ({gx:.1f},{gy:.1f})\n"
            f"{'='*60}")

        for run_num in range(1, OBSTACLE_RUNS + 1):
            self.get_logger().info(
                f"--- 第 {run_num}/{OBSTACLE_RUNS} 次 ---")

            # 导航到起点（无需轨迹）
            self.get_logger().info(
                f"  导航到起点: ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.0f}°)")
            if not self._send_goal_and_wait(sx, sy, syaw):
                self.get_logger().error(f"无法到达起点 {label}，跳过本次。")
                continue

            # 导航到终点 + 采样轨迹（用于计算 d_min）
            self.get_logger().info(
                f"  导航到终点（记录轨迹）: ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f}°)")
            trajectory, duration_s = self._send_goal_and_log_trajectory(
                gx, gy, gyaw, sample_rate_hz=OBS_SAMPLE_RATE_HZ
            )
            if trajectory is None:
                self.get_logger().error(f"路径 {label} 第{run_num}次导航失败。")

            # ── 计算 d_min（SLAM 轨迹 + 障碍物几何） ──────────────────
            if trajectory and len(trajectory) > 0:
                d_min_mm = self._compute_min_clearance(trajectory) * 1000.0
                self.get_logger().info(
                    f"  [自动] 最小轮廓间隙 d_min = {d_min_mm:.1f} mm  "
                    f"(基于 {len(trajectory)} 个 {OBS_SAMPLE_RATE_HZ:.0f}Hz 轨迹点)")
            else:
                d_min_mm = float("nan")
                self.get_logger().error("无轨迹数据，d_min 不可用。")

            # 写入避障轨迹 CSV
            if trajectory and len(trajectory) > 0:
                self._write_obstacle_trajectory_csv(
                    trajectory, label, run_num)

            # 记录 SLAM 稳态位姿
            cp = self._get_current_pose()
            if cp is None:
                self.get_logger().error("到达后定位丢失，无法记录位姿。")
                slam_x = slam_y = slam_yaw = float("nan")
            else:
                slam_x, slam_y, slam_yaw = cp
                self.get_logger().info(
                    f"  SLAM 到达位姿: ({slam_x:.3f}, {slam_y:.3f}, "
                    f"{math.degrees(slam_yaw):.1f}°)"
                )

            # ── 终端交互：人工输入碰撞次数 ──────────────────────────
            self.get_logger().info(
                f"  >>> 路径 {label} 第{run_num}次: "
                "请输入 collision (0=无碰撞, 1=发生碰撞):")
            collision_str = input("  collision (0/1): ").strip()
            try:
                collision = int(collision_str)
            except ValueError:
                collision = -1

            # ── 计算目标到达精度 ────────────────────────────────────
            if cp is not None:
                pos_err = math.sqrt((slam_x - gx) ** 2 + (slam_y - gy) ** 2)
                yaw_err = abs(normalize_angle(slam_yaw - gyaw))
            else:
                pos_err = float("nan")
                yaw_err = float("nan")

            self.get_logger().info(
                f"  目标到达精度: pos_err={pos_err:.4f} m  "
                f"yaw_err={math.degrees(yaw_err):.2f}°")

            # ── 写入 CSV ────────────────────────────────────────────
            self._write_obstacle_csv(
                label, run_num, collision, d_min_mm,
                pos_err, yaw_err, slam_x, slam_y, slam_yaw
            )

        self.get_logger().info("避障测试完成 ✓")

    # ══════════════════════════════════════════════════════════════════════
    # 最小轮廓间隙计算
    # ══════════════════════════════════════════════════════════════════════

    @staticmethod
    def _point_to_rect_distance(px: float, py: float,
                                rx0: float, ry0: float,
                                rx1: float, ry1: float) -> float:
        """点到轴对齐矩形的最短距离。若点在矩形内部返回 0。"""
        if px < rx0:
            dx = rx0 - px
        elif px > rx1:
            dx = px - rx1
        else:
            dx = 0.0

        if py < ry0:
            dy = ry0 - py
        elif py > ry1:
            dy = py - ry1
        else:
            dy = 0.0

        return math.sqrt(dx * dx + dy * dy)

    def _compute_min_clearance(self, trajectory):
        """从 SLAM 轨迹中计算车体包络矩形到障碍物箱的最小距离 (m)。

        对每个轨迹点 (t, x, y, yaw)，将车的 4 个角点变换到世界坐标，
        计算每个角点到障碍物矩形的距离，取全程最小值。
        """
        rx0, rx1 = OBSTACLE_BOX["x_min"], OBSTACLE_BOX["x_max"]
        ry0, ry1 = OBSTACLE_BOX["y_min"], OBSTACLE_BOX["y_max"]

        min_dist = float("inf")

        for (_t, cx, cy, cyaw) in trajectory:
            cos_yaw = math.cos(cyaw)
            sin_yaw = math.sin(cyaw)

            # 车体 4 角在局部坐标系: (±half_l, ±half_w)
            corners = [
                (cx + CAR_HALF_L * cos_yaw - CAR_HALF_W * sin_yaw,
                 cy + CAR_HALF_L * sin_yaw + CAR_HALF_W * cos_yaw),
                (cx + CAR_HALF_L * cos_yaw + CAR_HALF_W * sin_yaw,
                 cy + CAR_HALF_L * sin_yaw - CAR_HALF_W * cos_yaw),
                (cx - CAR_HALF_L * cos_yaw - CAR_HALF_W * sin_yaw,
                 cy - CAR_HALF_L * sin_yaw + CAR_HALF_W * cos_yaw),
                (cx - CAR_HALF_L * cos_yaw + CAR_HALF_W * sin_yaw,
                 cy - CAR_HALF_L * sin_yaw - CAR_HALF_W * cos_yaw),
            ]

            for wx, wy in corners:
                d = self._point_to_rect_distance(wx, wy, rx0, ry0, rx1, ry1)
                if d < min_dist:
                    min_dist = d

        return min_dist if min_dist != float("inf") else float("nan")

    # ══════════════════════════════════════════════════════════════════════
    # 导航到目标点（无轨迹记录）
    # ══════════════════════════════════════════════════════════════════════

    def _send_goal_and_wait(self, x, y, yaw):
        """发送导航目标并等待完成。返回 True 表示成功到达。"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("目标被 Nav2 拒绝")
            return False

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        last_localized_time = start_time

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            cp = self._get_current_pose()
            if cp is not None:
                last_localized_time = now
            elif now - last_localized_time > LOCALIZATION_LOST_TIMEOUT:
                self.get_logger().error(
                    f"定位丢失 {LOCALIZATION_LOST_TIMEOUT:.0f}s — 取消目标")
                goal_handle.cancel_goal_async()
                self._active_goal_handle = None
                return False

            if now - start_time > GOAL_TIMEOUT_SEC:
                self.get_logger().error(
                    f"目标超时 {GOAL_TIMEOUT_SEC:.0f}s — 取消")
                goal_handle.cancel_goal_async()
                self._active_goal_handle = None
                return False

        self._active_goal_handle = None
        result = result_future.result()
        if result is None:
            self.get_logger().error("目标执行失败")
            return False

        self.get_logger().info("  目标已到达 ✓")
        return True

    # ══════════════════════════════════════════════════════════════════════
    # 导航到目标点 + 轨迹采样（CTE 专用）
    # ══════════════════════════════════════════════════════════════════════

    def _send_goal_and_log_trajectory(self, x, y, yaw, sample_rate_hz: float):
        """导航的同时以固定频率采样 SLAM 位姿。
        返回 (trajectory, duration_s)，trajectory 为 [(t, x, y, yaw), ...] 列表。
        失败返回 (None, 0)。
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("目标被 Nav2 拒绝")
            return None, 0

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        start_time = time.time()
        last_localized_time = start_time

        trajectory = []
        sample_interval = 1.0 / sample_rate_hz
        last_sample_time = start_time - sample_interval  # 立即开始第一次采样

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.time()

            cp = self._get_current_pose()
            if cp is not None:
                last_localized_time = now

                # ── 按频率采样 ────────────────────────────────────────
                if now - last_sample_time >= sample_interval:
                    trajectory.append((now - start_time, cp[0], cp[1], cp[2]))
                    last_sample_time = now

            elif now - last_localized_time > LOCALIZATION_LOST_TIMEOUT:
                self.get_logger().error(
                    f"定位丢失 {LOCALIZATION_LOST_TIMEOUT:.0f}s — 取消目标")
                goal_handle.cancel_goal_async()
                self._active_goal_handle = None
                # 返回已采集的部分轨迹（仍可用于部分分析）
                if trajectory:
                    return trajectory, trajectory[-1][0]
                return None, 0

            if now - start_time > GOAL_TIMEOUT_SEC:
                self.get_logger().error(
                    f"目标超时 {GOAL_TIMEOUT_SEC:.0f}s — 取消")
                goal_handle.cancel_goal_async()
                self._active_goal_handle = None
                if trajectory:
                    return trajectory, trajectory[-1][0]
                return None, 0

        self._active_goal_handle = None
        result = result_future.result()
        if result is None:
            self.get_logger().error("目标执行失败")
            if trajectory:
                return trajectory, trajectory[-1][0]
            return None, 0

        # ── 到达后补录最后一帧（如果间隔不足） ──────────────────────────
        cp = self._get_current_pose()
        if cp is not None:
            final_t = time.time() - start_time
            last_t = trajectory[-1][0] if trajectory else -999
            if final_t - last_t >= sample_interval * 0.5:  # 避免重复过近
                trajectory.append((final_t, cp[0], cp[1], cp[2]))

        duration_s = trajectory[-1][0] if trajectory else time.time() - start_time
        self.get_logger().info(
            f"  目标已到达 ✓  轨迹点数: {len(trajectory)}  耗时: {duration_s:.1f}s")
        return trajectory, duration_s

    # ══════════════════════════════════════════════════════════════════════
    # CTE CSV 输出
    # ══════════════════════════════════════════════════════════════════════

    def _write_cte_csv(self, seg_results, all_rmse, all_max, all_N, all_dur):
        """写入 CTE 汇总结果到 cte_results.csv（AppendingCSVLogger）。

        seg_results: [(label, traj, cte_list, rmse, max_cte, n, dur), ...] ×4 段
        """
        headers = [
            "timestamp",
            # 逐段
            "seg_1to2_rmse_m", "seg_1to2_max_m", "seg_1to2_n", "seg_1to2_dur_s",
            "seg_2to3_rmse_m", "seg_2to3_max_m", "seg_2to3_n", "seg_2to3_dur_s",
            "seg_3to4_rmse_m", "seg_3to4_max_m", "seg_3to4_n", "seg_3to4_dur_s",
            "seg_4to1_rmse_m", "seg_4to1_max_m", "seg_4to1_n", "seg_4to1_dur_s",
            # 整体
            "overall_rmse_m", "overall_max_m", "overall_n", "overall_dur_s",
            "notes",
        ]

        logger = AppendingCSVLogger(CSV_CTE_PATH, headers)

        row = [f"{time.time():.3f}"]
        # 逐段（按 label 顺序: 1→2, 2→3, 3→4, 4→1）
        for _, _, _, rmse, max_cte, n, dur in seg_results:
            row.extend([f"{rmse:.4f}", f"{max_cte:.4f}", f"{n}", f"{dur:.2f}"])
        # 整体
        row.extend([f"{all_rmse:.4f}", f"{all_max:.4f}", f"{all_N}", f"{all_dur:.2f}", ""])

        logger.add_row(row)
        logger.close()

        self.get_logger().info(
            f"CTE 结果已写入 {CSV_CTE_PATH} (run_id={logger.run_id})")

    def _write_trajectory_csv(self, trajectory, cte_values, segment_label: str = ""):
        """写入完整轨迹到 trajectory_cte_{label}_{timestamp}.csv（CSVLogger）。"""
        headers = ["t_s", "x_m", "y_m", "yaw_rad", "cte_m"]

        safe_label = segment_label.replace("→", "_to_") if segment_label else "cte"
        csv_logger = CSVLogger(TRAJECTORY_LOG_DIR, f"trajectory_cte_{safe_label}", headers)
        self.get_logger().info(f"轨迹日志 -> {csv_logger.filepath}")

        for (t_val, px, py, pyaw), cte in zip(trajectory, cte_values):
            csv_logger.add_row([
                f"{t_val:.3f}",
                f"{px:.4f}",
                f"{py:.4f}",
                f"{pyaw:.4f}",
                f"{cte:.4f}",
            ])

        csv_logger.close()

    # ══════════════════════════════════════════════════════════════════════
    # 避障 CSV 输出
    # ══════════════════════════════════════════════════════════════════════

    def _write_obstacle_trajectory_csv(self, trajectory, path_label, run_num):
        """写入避障轨迹到 trajectory_obs_<timestamp>.csv（CSVLogger）。"""
        headers = ["t_s", "x_m", "y_m", "yaw_rad"]

        safe_label = path_label.replace("→", "_to_")
        csv_logger = CSVLogger(
            TRAJECTORY_LOG_DIR,
            f"trajectory_obs_{safe_label}_run{run_num}",
            headers)
        self.get_logger().info(f"避障轨迹日志 -> {csv_logger.filepath}")

        for t_val, px, py, pyaw in trajectory:
            csv_logger.add_row([
                f"{t_val:.3f}",
                f"{px:.4f}",
                f"{py:.4f}",
                f"{pyaw:.4f}",
            ])

        csv_logger.close()

    def _write_obstacle_csv(self, path_label, run_num, collision, d_min_mm,
                            pos_err, yaw_err, slam_x, slam_y, slam_yaw):
        """写入避障测试结果到 obstacle_avoidance_results.csv（AppendingCSVLogger）。"""
        headers = [
            "timestamp",
            "path_label",
            "run_num",
            "collision",
            "d_min_mm",
            "goal_arrival_pos_err_m",
            "goal_arrival_yaw_err_deg",
            "slam_x",
            "slam_y",
            "slam_yaw_deg",
            "notes",
        ]

        logger = AppendingCSVLogger(CSV_OBSTACLE_PATH, headers)

        row = [
            f"{time.time():.3f}",
            path_label,
            f"{run_num}",
            f"{collision}",
            f"{d_min_mm:.1f}" if d_min_mm >= 0 else "",
            f"{pos_err:.4f}" if not math.isnan(pos_err) else "",
            f"{math.degrees(yaw_err):.4f}" if not math.isnan(yaw_err) else "",
            f"{slam_x:.4f}" if not math.isnan(slam_x) else "",
            f"{slam_y:.4f}" if not math.isnan(slam_y) else "",
            f"{math.degrees(slam_yaw):.4f}" if not math.isnan(slam_yaw) else "",
            "",
        ]

        logger.add_row(row)
        logger.close()

        self.get_logger().info(
            f"避障结果已写入 {CSV_OBSTACLE_PATH} (run_id={logger.run_id})")


# ═══════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Test 2 — 导航控制与避障指标测试"
    )
    parser.add_argument(
        "--mode", type=str, default="all",
        choices=["cte", "obstacle", "all"],
        help="测试模式: cte=直线跟踪, obstacle=避障(需指定--path), all=全部（仅CTE）"
    )
    parser.add_argument(
        "--path", type=str, default="",
        choices=["1to3", "4to2", ""],
        help="避障路径（仅在 --mode obstacle 时有效）: 1to3 | 4to2"
    )
    args = parser.parse_args()

    if args.mode == "obstacle" and not args.path:
        parser.error("--mode obstacle 必须指定 --path 1to3 或 --path 4to2")

    rclpy.init(args=sys.argv)
    node = NavCteObstacleTest(mode=args.mode, obstacle_path=args.path)
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
