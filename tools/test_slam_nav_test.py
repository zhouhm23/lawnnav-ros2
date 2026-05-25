#!/usr/bin/env python3
"""
test_slam_nav_test.py — 三种传感器 SLAM 定位综合测试（RPE + CTE + 静态稳定性 + 碰撞）。

用法:
    python3 tools/test_slam_nav_test.py --sensor camera|lidar|vslam

测试流程:
    1. 等待定位就绪（自动检测 sensor 对应的 pose topic）
    2. 导航闭合矩形 1→2→3→4→1
    3. 每段自动记录轨迹并计算 CTE（cross-track error）
    4. 航点 3, 4, 1 处暂停：
       - 自动采集 5s 静态稳定性数据
       - 等待用户输入论文坐标系地面真值
       - (仅 3→4 段) 手动输入是否碰撞
    5. 计算 RPE + 闭合误差
    6. 原始数据 → logs/pose/   论文数据 → tools/results/

传感器对应 pose topic:
    camera : /rtabmap/localization_pose
    lidar  : /amcl_pose
    vslam  : /rtabmap/localization_pose
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
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
STATIC_DURATION_SEC = 5.0        # 航点处静态采集时长
STATIC_RATE_HZ = 5.0
CTE_SAMPLE_HZ = 2.0              # 导航途中 CTE 采样频率

# 各传感器对应的定位 topic
SENSOR_POSE_TOPIC = {
    "camera": "/rtabmap/localization_pose",
    "lidar":  "/amcl_pose",
    "vslam":  "/rtabmap/localization_pose",
}

# 导航路径（map 坐标系，x⁺=车头, y⁺=左侧）
# 矩形: 1(0,0) → 2(1.8,0) → 3(1.8,-1.0) → 4(0,-1.0) → 1(0,0)
GOALS_MAP = [
    (0.0,   0.0,  0.0),              # 点1 (起点)
    (1.8,   0.0,  0.0),              # 点2
    (1.8,  -1.0,  -math.pi / 2.0),   # 点3
    (0.0,  -1.0,  math.pi / 2.0),    # 点4
    (0.0,   0.0,  0.0),              # 点1 (回到起点, 闭合)
]
PAPER_LABELS = [1, 2, 3, 4, 1]       # 论文点编号

# 需要进行手动 GT 输入的航点索引（0-based）及其论文坐标系理论值
# 用户输入偏差 (dx dy yaw_actual)，如 0.02 -0.04 176 → 实际=(理论+偏差)
GT_WAYPOINTS = {
    2: (1.4, 1.8, 180.0),   # 点3 理论论文坐标
    3: (1.4, 0.0, 270.0),    # 点4
    4: (0.4, 0.0, 0.0),      # 点1 (回到起点)
}

# 输出目录
RESULTS_DIR = str(Path(__file__).resolve().parent / "results")
RAW_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/pose"
CSV_RPE_PATH = os.path.join(RESULTS_DIR, "slam_rpe_results.csv")
CSV_CTE_PATH = os.path.join(RESULTS_DIR, "slam_cte_results.csv")
CSV_STATIC_PATH = os.path.join(RESULTS_DIR, "slam_static_results.csv")

os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(RAW_LOG_DIR, exist_ok=True)


# ═══════════════════════════════════════════════════════════════════════════
# 坐标系转换 (paper ↔ map)
# ═══════════════════════════════════════════════════════════════════════════

def paper_to_map(x_p: float, y_p: float, yaw_deg_p: float = 0.0):
    x_m = y_p
    y_m = -x_p + 0.4
    yaw_m = normalize_angle(math.radians(yaw_deg_p) - math.pi / 2.0)
    return (x_m, y_m, yaw_m)


def map_to_paper(x_m: float, y_m: float, yaw_m: float = 0.0):
    x_p = -y_m + 0.4
    y_p = x_m
    yaw_deg_p = math.degrees(normalize_angle(yaw_m + math.pi / 2.0))
    return (x_p, y_p, yaw_deg_p)


# ═══════════════════════════════════════════════════════════════════════════
# SlamNavTest 节点
# ═══════════════════════════════════════════════════════════════════════════

class SlamNavTest(Node):
    def __init__(self, sensor: str = "camera"):
        super().__init__("slam_nav_test")

        self._sensor = sensor
        self._pose_topic = SENSOR_POSE_TOPIC.get(sensor, SENSOR_POSE_TOPIC["camera"])
        self._pose_msg = None
        self._active_pose_topic = None

        self.get_logger().info(f"传感器: {sensor}  ->  定位 topic: {self._pose_topic}")

        # ── 定位订阅 ────────────────────────────────────────────────────
        self.create_subscription(
            PoseWithCovarianceStamped, self._pose_topic,
            self._pose_callback, 10)

        # ── Action client ────────────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── 数据存储 ────────────────────────────────────────────────────
        self._slam_poses = []        # [(x, y, yaw)] 每个航点到达时
        self._gt_poses_paper = []    # 用户输入 (x_p, y_p, yaw_deg)
        self._gt_poses_map = []      # 转换后 (x, y, yaw)
        self._segment_trajectories = []  # [[(x,y,yaw)], ...]  每段原始轨迹
        self._segment_cte = []       # [cte_rmse, cte_max] 每段
        self._collisions = []        # [0/1, ...]

    # ── 回调 ─────────────────────────────────────────────────────────

    def _pose_callback(self, msg):
        self._pose_msg = msg
        if self._active_pose_topic is None:
            self._active_pose_topic = self._pose_topic
            self.get_logger().info(f"定位源已激活: {self._pose_topic}")

    # ── 位姿辅助 ─────────────────────────────────────────────────────

    def _get_current_pose(self):
        if self._pose_msg is None:
            return None
        p = self._pose_msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        return (p.position.x, p.position.y, yaw)

    def _get_current_stamp(self):
        if self._pose_msg is None:
            return (0, 0)
        return (self._pose_msg.header.stamp.sec, self._pose_msg.header.stamp.nanosec)

    # ══════════════════════════════════════════════════════════════════════
    # 主入口
    # ══════════════════════════════════════════════════════════════════════

    def run(self):
        # 等待定位
        self.get_logger().info(f"等待定位数据 (topic: {self._pose_topic})...")
        deadline = time.time() + 20.0
        while self._active_pose_topic is None and rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._active_pose_topic is None:
            self.get_logger().error(f"20s 内无定位数据！检查: {self._pose_topic}")
            return

        # 打印初始位姿
        cp = self._get_current_pose()
        if cp is not None:
            x_p, y_p, yaw_deg_p = map_to_paper(cp[0], cp[1], cp[2])
            self.get_logger().info(f"初始位姿 map:({cp[0]:.3f},{cp[1]:.3f},{math.degrees(cp[2]):.1f}°)  "
                                   f"论文:({x_p:.3f},{y_p:.3f},{yaw_deg_p:.1f}°)")

        # 等待 Nav2
        self.get_logger().info("等待 Nav2 action server...")
        self._action_client.wait_for_server()

        # ── 导航闭合矩形 ────────────────────────────────────────────────
        for idx in range(len(GOALS_MAP)):
            gx, gy, gyaw = GOALS_MAP[idx]
            label = PAPER_LABELS[idx]
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"航点 {label} — map:({gx:.2f},{gy:.2f},{math.degrees(gyaw):.0f}°)")

            # 导航到目标 + 沿途采样轨迹
            if idx == 0:
                # 起点：只需导航到达，不记录轨迹
                ok = self._send_goal_and_wait(gx, gy, gyaw, record_traj=False)
            else:
                ok = self._send_goal_and_wait(gx, gy, gyaw, record_traj=True)

            if not ok:
                self.get_logger().error(f"航点 {label} 导航失败，测试中止")
                self._write_results()
                return

            # 记录到达位姿
            cp = self._get_current_pose()
            if cp is None:
                self.get_logger().error(f"航点 {label} 到达后定位丢失")
                self._write_results()
                return
            self._slam_poses.append(cp)
            x_p, y_p, yaw_deg_p = map_to_paper(cp[0], cp[1], cp[2])
            self.get_logger().info(f"  到达点{label} SLAM(map):({cp[0]:.3f},{cp[1]:.3f},{math.degrees(cp[2]):.1f}°)  "
                                   f"SLAM(论文):({x_p:.3f},{y_p:.3f},{yaw_deg_p:.1f}°)")

            # 计算上一段 CTE（第一段跳过）
            if idx > 0:
                self._compute_segment_cte(idx - 1)

            # 航点 3, 4, 1(闭合) 处暂停
            if idx in GT_WAYPOINTS:
                # 静态稳定性采集
                self._collect_static_at_waypoint(label)

                # 地面真值输入（偏差格式）
                exp_x, exp_y, exp_yaw = GT_WAYPOINTS[idx]
                self.get_logger().info(
                    f"  >>> 点{label} 理论论文坐标: ({exp_x}, {exp_y}, {exp_yaw}°)")
                self.get_logger().info(
                    f"      输入偏差 (dx dy yaw_actual°)，如: 0.02 -0.04 176")
                gt_str = input("  GT (dx dy yaw_actual): ").strip()
                try:
                    parts = gt_str.split()
                    dx = float(parts[0])
                    dy = float(parts[1])
                    yaw_actual = float(parts[2]) if len(parts) > 2 else exp_yaw
                except (ValueError, IndexError):
                    self.get_logger().error(f"输入格式错误: '{gt_str}'，预期 'dx dy yaw_actual'")
                    self._write_results()
                    return

                gt_x_p = exp_x + dx
                gt_y_p = exp_y + dy
                gt_yaw_deg_p = yaw_actual
                self.get_logger().info(
                    f"  实际论文坐标: ({gt_x_p:.3f}, {gt_y_p:.3f}, {gt_yaw_deg_p:.1f}°)")

                self._gt_poses_paper.append((gt_x_p, gt_y_p, gt_yaw_deg_p))
                gt_map = paper_to_map(gt_x_p, gt_y_p, gt_yaw_deg_p)
                self._gt_poses_map.append(gt_map)

                # 仅点3→4段：手动输入碰撞
                if idx == 3:  # 在点4，询问点3→4之间是否碰撞
                    self.get_logger().info("  >>> 点3→4 是否碰撞? (0=无碰撞 1=碰撞):")
                    coll_str = input("  碰撞 (0/1): ").strip()
                    self._collisions.append(int(coll_str) if coll_str in ("0", "1") else 0)
                else:
                    self._collisions.append(0)  # 其他段不关心碰撞

        # ── 计算并输出 ──────────────────────────────────────────────────
        self._compute_and_log_rpe()
        self._write_results()
        self.get_logger().info("所有测试完成 ✓")

    # ══════════════════════════════════════════════════════════════════════
    # 导航
    # ══════════════════════════════════════════════════════════════════════

    def _send_goal_and_wait(self, x, y, yaw, record_traj=False):
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
        traj = []  # 沿途轨迹采样

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            cp = self._get_current_pose()
            if cp is not None:
                last_localized_time = now
                if record_traj:
                    # 以 CTE_SAMPLE_HZ 采样
                    if not traj or now - traj[-1][0] >= 1.0 / CTE_SAMPLE_HZ:
                        traj.append((now, cp[0], cp[1], cp[2]))
            elif now - last_localized_time > LOCALIZATION_LOST_TIMEOUT:
                self.get_logger().error(f"定位丢失 {LOCALIZATION_LOST_TIMEOUT:.0f}s — 取消目标")
                goal_handle.cancel_goal_async()
                return False

            if now - start_time > GOAL_TIMEOUT_SEC:
                self.get_logger().error(f"目标超时 {GOAL_TIMEOUT_SEC:.0f}s — 取消")
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        if result is None:
            self.get_logger().error("目标执行失败")
            return False

        if record_traj and traj:
            self._segment_trajectories.append(traj)
            self.get_logger().info(f"  轨迹采样: {len(traj)} 点")

        self.get_logger().info("  目标已到达 ✓")
        return True

    # ══════════════════════════════════════════════════════════════════════
    # CTE 计算（线段 cross-track error）
    # ══════════════════════════════════════════════════════════════════════

    def _compute_segment_cte(self, seg_idx):
        """计算 seg_idx 段 (起点 GOALS_MAP[seg_idx] → GOALS_MAP[seg_idx+1]) 的 CTE。"""
        if seg_idx >= len(self._segment_trajectories):
            return

        traj = self._segment_trajectories[seg_idx]
        if len(traj) < 2:
            self._segment_cte.append((float("nan"), float("nan")))
            return

        # 理想线段: 起点→终点
        sx, sy, _ = GOALS_MAP[seg_idx]
        ex, ey, _ = GOALS_MAP[seg_idx + 1]
        seg_dx = ex - sx
        seg_dy = ey - sy
        seg_len = math.sqrt(seg_dx ** 2 + seg_dy ** 2)

        if seg_len < 1e-6:
            # 零长度线段（如起点=终点），使用点到点距离
            errors = [math.sqrt((p[1] - sx) ** 2 + (p[2] - sy) ** 2)
                      for p in traj]
        else:
            # cross-track error = |(p - start) × seg_dir|
            ux = seg_dx / seg_len
            uy = seg_dy / seg_len
            errors = []
            for _, px, py, _ in traj:
                dx = px - sx
                dy = py - sy
                # 投影长度
                proj = dx * ux + dy * uy
                # 最近点
                cx = sx + proj * ux
                cy = sy + proj * uy
                cte = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)
                errors.append(cte)

        cte_rmse = math.sqrt(sum(e ** 2 for e in errors) / len(errors))
        cte_max = max(errors)

        self._segment_cte.append((cte_rmse, cte_max))
        labels = [(1, 2), (2, 3), (3, 4), (4, 1)]
        self.get_logger().info(f"  CTE {labels[seg_idx][0]}→{labels[seg_idx][1]}: "
                               f"RMSE={cte_rmse:.4f}m  MAX={cte_max:.4f}m")

    # ══════════════════════════════════════════════════════════════════════
    # 静态稳定性（在航点暂停时采集）
    # ══════════════════════════════════════════════════════════════════════

    def _collect_static_at_waypoint(self, label):
        self.get_logger().info(f"  >>> 点{label} 静态稳定性采集 {STATIC_DURATION_SEC:.0f}s...")

        # 稳定等待
        settle_deadline = time.monotonic() + 2.0
        while time.monotonic() < settle_deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        interval = 1.0 / STATIC_RATE_HZ
        start_mono = time.monotonic()
        samples = []
        last_stamp_ns = -1

        while time.monotonic() - start_mono < STATIC_DURATION_SEC and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            cp = self._get_current_pose()
            if cp is not None:
                stamp_sec, stamp_ns = self._get_current_stamp()
                stamp_key = stamp_sec * 10**9 + stamp_ns
                if stamp_key == last_stamp_ns:
                    continue
                last_stamp_ns = stamp_key
                samples.append((cp[0], cp[1], cp[2]))
            time.sleep(interval)

        if len(samples) < 2:
            self.get_logger().warn(f"  点{label} 静态样本不足")
            return

        # 以第一帧为参考计算漂移
        ref_x, ref_y, ref_yaw = samples[0]
        deviations = [math.sqrt((x - ref_x) ** 2 + (y - ref_y) ** 2)
                      for x, y, _ in samples]
        yaw_devs = [abs(normalize_angle(yaw - ref_yaw)) for _, _, yaw in samples]

        rmse_pos = math.sqrt(sum(d ** 2 for d in deviations) / len(deviations))
        rmse_yaw = math.sqrt(sum(d ** 2 for d in yaw_devs) / len(yaw_devs))
        max_pos = max(deviations)
        max_yaw = max(yaw_devs)

        self.get_logger().info(f"  点{label} 静态 RMSE: pos={rmse_pos*1e3:.1f}mm  yaw={math.degrees(rmse_yaw):.3f}°  "
                               f"MAX: pos={max_pos*1e3:.1f}mm  yaw={math.degrees(max_yaw):.3f}°")

        # 保存原始采样
        self._save_raw_static(label, samples)

        # 保存汇总结果
        self._save_static_result(label, len(samples), rmse_pos, rmse_yaw, max_pos, max_yaw)

    def _save_raw_static(self, label, samples):
        csv_logger = CSVLogger(
            RAW_LOG_DIR, f"static_{self._sensor}_pt{label}",
            ["x", "y", "yaw"])
        for x, y, yaw in samples:
            csv_logger.add_row([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])
        csv_logger.close()

    def _save_static_result(self, label, n_samples, rmse_pos, rmse_yaw, max_pos, max_yaw):
        headers = ["timestamp", "sensor", "waypoint", "n_samples",
                   "rmse_pos_m", "rmse_yaw_deg", "max_pos_m", "max_yaw_deg"]
        logger = AppendingCSVLogger(CSV_STATIC_PATH, headers)
        logger.add_row([
            f"{time.time():.3f}", self._sensor, str(label), str(n_samples),
            f"{rmse_pos:.6f}", f"{math.degrees(rmse_yaw):.6f}",
            f"{max_pos:.6f}", f"{math.degrees(max_yaw):.6f}",
        ])
        logger.close()

    # ══════════════════════════════════════════════════════════════════════
    # RPE 计算
    # ══════════════════════════════════════════════════════════════════════

    def _compute_and_log_rpe(self):
        N = min(len(self._slam_poses), len(self._gt_poses_map))
        if N < 2:
            self.get_logger().error("样本不足，无法计算 RPE")
            return

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("            闭合路径 RPE 汇总")
        self.get_logger().info("=" * 60)

        # 位姿对照
        self.get_logger().info(f"{'点':>4} {'SLAM_x':>8} {'SLAM_y':>8} {'SLAM_yaw°':>8}  "
                               f"{'GT_x':>8} {'GT_y':>8} {'GT_yaw°':>8}")
        for i in range(N):
            sx, sy, syaw = self._slam_poses[i + 1] if i + 1 < len(self._slam_poses) else self._slam_poses[0]
            gx, gy, gyaw = self._gt_poses_map[i]
            labels = [3, 4, 1]  # GT 对应论文点
            self.get_logger().info(f"{labels[i]:>4} {sx:>8.3f} {sy:>8.3f} {math.degrees(syaw):>8.1f}  "
                                   f"{gx:>8.3f} {gy:>8.3f} {math.degrees(gyaw):>8.1f}")

        # 逐段 RPE
        self._save_rpe_results()

    def _save_rpe_results(self):
        N = len(self._gt_poses_map)
        if N < 2:
            return

        slam_all = self._slam_poses  # [pt1, pt2, pt3, pt4, pt1]
        # GT 仅点 3, 4, 1; 需要从路径中提取对应 SLAM
        gt_map = self._gt_poses_map  # [(pt3), (pt4), (pt1)]

        # 对应 SLAM: 点3=slam[2], 点4=slam[3], 点1(回到)=slam[4]
        slam_corresponding = [slam_all[2], slam_all[3], slam_all[4]]

        seg_errors_pos = []
        seg_errors_yaw = []

        for i in range(N):
            j = (i + 1) % N
            sx0, sy0, syaw0 = slam_corresponding[i]
            sx1, sy1, syaw1 = slam_corresponding[j]
            gx0, gy0, gyaw0 = gt_map[i]
            gx1, gy1, gyaw1 = gt_map[j]

            ds_x = sx1 - sx0
            ds_y = sy1 - sy0
            ds_yaw = normalize_angle(syaw1 - syaw0)
            dg_x = gx1 - gx0
            dg_y = gy1 - gy0
            dg_yaw = normalize_angle(gyaw1 - gyaw0)

            e_pos = math.sqrt((ds_x - dg_x) ** 2 + (ds_y - dg_y) ** 2)
            e_yaw = abs(normalize_angle(ds_yaw - dg_yaw))
            seg_errors_pos.append(e_pos)
            seg_errors_yaw.append(e_yaw)

        mean_pos = sum(seg_errors_pos) / len(seg_errors_pos)
        mean_yaw = sum(seg_errors_yaw) / len(seg_errors_yaw)

        self.get_logger().info(f"\nRPE 汇总: 位置 {mean_pos:.4f}±{math.sqrt(sum((e-mean_pos)**2 for e in seg_errors_pos)/len(seg_errors_pos)):.4f} m  "
                               f"航向 {math.degrees(mean_yaw):.2f}°")

        # 写入 CSV
        headers = ["timestamp", "sensor",
                   "seg_3to4_pos_m", "seg_3to4_yaw_deg",
                   "seg_4to1_pos_m", "seg_4to1_yaw_deg",
                   "seg_1to3_pos_m", "seg_1to3_yaw_deg",
                   "mean_pos_m", "mean_yaw_deg"]
        logger = AppendingCSVLogger(CSV_RPE_PATH, headers)
        row = [f"{time.time():.3f}", self._sensor]
        for i in range(3):
            row.append(f"{seg_errors_pos[i]:.4f}")
            row.append(f"{math.degrees(seg_errors_yaw[i]):.4f}")
        row.append(f"{mean_pos:.4f}")
        row.append(f"{math.degrees(mean_yaw):.4f}")
        logger.add_row(row)
        logger.close()

    # ══════════════════════════════════════════════════════════════════════
    # 结果写入
    # ══════════════════════════════════════════════════════════════════════

    def _write_results(self):
        # CTE
        if self._segment_cte:
            headers = ["timestamp", "sensor",
                       "cte_1to2_rmse_m", "cte_1to2_max_m",
                       "cte_2to3_rmse_m", "cte_2to3_max_m",
                       "cte_3to4_rmse_m", "cte_3to4_max_m",
                       "cte_4to1_rmse_m", "cte_4to1_max_m"]
            logger = AppendingCSVLogger(CSV_CTE_PATH, headers)
            row = [f"{time.time():.3f}", self._sensor]
            for i in range(4):
                if i < len(self._segment_cte):
                    row.append(f"{self._segment_cte[i][0]:.4f}")
                    row.append(f"{self._segment_cte[i][1]:.4f}")
                else:
                    row.extend(["", ""])
            logger.add_row(row)
            logger.close()

        self.get_logger().info(f"\n论文数据已保存到 {RESULTS_DIR}/")
        self.get_logger().info(f"  RPE:    {CSV_RPE_PATH}")
        self.get_logger().info(f"  CTE:    {CSV_CTE_PATH}")
        self.get_logger().info(f"  Static: {CSV_STATIC_PATH}")
        self.get_logger().info(f"原始轨迹 -> {RAW_LOG_DIR}/")


# ═══════════════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="三种传感器 SLAM 综合测试")
    parser.add_argument("--sensor", choices=["camera", "lidar", "vslam"],
                        default="camera", help="传感器配置")
    args = parser.parse_args()

    rclpy.init()
    node = SlamNavTest(sensor=args.sensor)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
