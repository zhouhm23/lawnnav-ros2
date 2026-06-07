#!/usr/bin/env python3
"""
test_slam_nav_test.py — 三种传感器 SLAM 定位综合测试（RPE + CTE + 静态稳定性 + 碰撞）。

用法:
    python3 tools/test_slam_nav_test.py --sensor camera|lidar|vslam

测试流程:
    1. 脚本自动停止旧进程 → 启动对应传感器的建图+导航 launch
    2. 等待定位 + Nav2 就绪
    3. 导航三角形 1→3→4→1（障碍物在 1→3 对角线路径上）
    4. 每段自动记录轨迹并计算 CTE
    5. 航点 3, 4, 1 处暂停：
       - 自动采集 5s 静态稳定性数据
       - 等待用户输入论文坐标系地面真值（偏差格式）
       - (仅 1→3 段) 手动输入是否碰撞
    6. 计算 RPE + 闭合误差 → 写入 CSV（仅全部成功时）
    7. 原始数据 → logs/pose/   论文数据 → tools/results/

传感器对应启动命令:
    camera : rtabmap_camera_nav.launch.py (mapping 模式, 内含 Nav2)
    lidar  : slam_toolbox_lidar_slam + slam_toolbox_lidar_nav (map:= 动态地图)
    vslam  : rtabmap_vslam_nav.launch.py (mapping 模式, 内含 Nav2)
"""

import argparse
import math
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from test_utils import (
    yaw_from_quaternion,
    make_pose_stamped,
    normalize_angle,
    CSVLogger,
    AppendingCSVLogger,
    sample_cpu_mem,
    save_perf_samples,
)

# ═══════════════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════════════

GOAL_TIMEOUT_SEC = 120.0
LOCALIZATION_LOST_TIMEOUT = 3.0
STATIC_DURATION_SEC = 15.0
STATIC_RATE_HZ = 10.0
PERF_SAMPLE_INTERVAL = 10.0   # 性能采样间隔 (秒)
PERF_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/slam_perf"
WP_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/waypoints"
TRAJ_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/trajectory"
CTE_SAMPLE_HZ = 2.0              # CTE 轨迹采样频率
NAV_STARTUP_WAIT = 15.0       # 建图+导航启动等待 (lidar 保底15s)
WS_ROOT = Path(__file__).resolve().parent.parent

# 各传感器独立的建图+导航启动命令
NAV_LAUNCH_CMD = {
    "camera": "ros2 launch navigation rtabmap_camera_nav.launch.py",
    "vslam":  "ros2 launch navigation rtabmap_vslam_nav.launch.py",
    # lidar 已合并为单 launch (localization:=false=建图, true=定位)
    "lidar":  "ros2 launch navigation slam_toolbox_lidar_nav.launch.py",
}

# 各传感器对应的定位 topic
SENSOR_POSE_TOPIC = {
    "camera": "/localization_pose",
    "lidar":  "/odom",
    "vslam":  "/localization_pose",
}

# 导航路径: 三角形 1→3→4→1（map 坐标系，x⁺=车头, y⁺=左侧）
GOALS_MAP = [
    (0.0,   0.0,  0.0),               # 点1 (起点, 不导航, 仅初始位姿)
    (1.8, -1.0, -math.pi),         # 论文点3: (1.4, 1.8)
    (0.0, -1.0,  math.pi / 2.0),   # 论文点4: (1.4, 0)
    (0.0,  0.0,  0.0),             # 论文点1: (0.4, 0)
]
PAPER_LABELS = [1, 3, 4, 1]
SEGMENT_NAMES = [(1, 3), (3, 4), (4, 1)]  # 3 段 CTE

# GT 输入航点 (0-based index) 及其论文坐标系理论值
# 用户输入偏差 (dx dy yaw_actual)，如 0.02 -0.04 176 → 实际=(理论+偏差)
GT_WAYPOINTS = {
    1: (1.4, 1.8, -90),   # 点3  理论论文坐标
    2: (1.4, 0.0, 180),    # 点4
    3: (0.4, 0.0, 90),      # 点1 (闭合)
}

# 输出目录
RESULTS_DIR = str(Path(__file__).resolve().parent / "results")
RAW_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/pose"
CSV_RPE_PATH = os.path.join(RESULTS_DIR, "slam_rpe_results.csv")
CSV_CTE_PATH = os.path.join(RESULTS_DIR, "slam_cte_results.csv")
CSV_STATIC_PATH = os.path.join(RESULTS_DIR, "slam_static_results.csv")
CSV_PERF_PATH = os.path.join(RESULTS_DIR, "slam_perf_results.csv")

os.makedirs(RESULTS_DIR, exist_ok=True)
os.makedirs(RAW_LOG_DIR, exist_ok=True)
os.makedirs(PERF_LOG_DIR, exist_ok=True)
os.makedirs(WP_LOG_DIR, exist_ok=True)
os.makedirs(TRAJ_LOG_DIR, exist_ok=True)


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
        self._nav_procs = []       # 脚本启动的子进程

        self.get_logger().info(f"传感器: {sensor}  ->  定位 topic: {self._pose_topic}")

        # lidar 的 /odom 是 nav_msgs/Odometry，其他是 PoseWithCovarianceStamped
        if sensor == "lidar":
            self.create_subscription(Odometry, self._pose_topic, self._pose_callback, 10)
        else:
            self.create_subscription(PoseWithCovarianceStamped, self._pose_topic, self._pose_callback, 10)

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._slam_poses = []
        self._gt_poses_paper = []
        self._gt_poses_map = []
        self._segment_trajectories = []
        self._segment_cte = []
        self._collisions = []
        self._static_results = []
        self._waypoint_rows = []    # 航点原始记录（含用户 GT 输入）
        self._static_samples = []     # 回调驱动的静态样本
        self._static_collecting = False
        self._cpu_samples = []
        self._mem_samples = []
        self._success = False

    # ── 回调 ─────────────────────────────────────────────────────────

    def _source_cmd(self):
        """返回 source ROS2 环境的 bash 命令前缀。"""
        parts = ["source /opt/ros/humble/setup.sh"]
        ws = WS_ROOT / "install" / "setup.bash"
        if ws.exists():
            parts.append(f"source {shlex.quote(str(ws))}")
        return " && ".join(parts)

    def _start_navigation(self):
        """根据 sensor 启动建图+导航 launch，阻塞等待 Nav2 就绪。"""
        # 停止旧进程
        s = Path.home() / ".stop_ros.sh"
        if s.exists():
            subprocess.call(["sudo", str(s)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(5)
        else:
            subprocess.call(["pkill", "-f", "ros2"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(5)

        # 统一启动: 所有传感器均使用单一 launch (lidar 已合并 SLAM+Nav)
        cmd = NAV_LAUNCH_CMD[self._sensor]
        self.get_logger().info(f"启动: {cmd}")
        proc = subprocess.Popen(
            ["bash", "-lc", f"{self._source_cmd()} && {cmd}"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._nav_procs.append(proc)

        self.get_logger().info(f"等待建图+导航就绪 ({NAV_STARTUP_WAIT:.0f}s)...")
        time.sleep(NAV_STARTUP_WAIT)

    def _stop_navigation(self):
        """清理脚本启动的子进程。"""
        for p in self._nav_procs:
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    p.kill()
        self._nav_procs = []

    # ── 回调 ─────────────────────────────────────────────────────────

    def _pose_callback(self, msg):
        self._pose_msg = msg
        if self._active_pose_topic is None:
            self._active_pose_topic = self._pose_topic
            self.get_logger().info(f"定位源已激活: {self._pose_topic}")
        # 静态采集期间自动记录
        if self._static_collecting:
            p = msg.pose.pose
            yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                      p.orientation.z, p.orientation.w)
            self._static_samples.append((p.position.x, p.position.y, yaw))

    # ── 位姿辅助 ─────────────────────────────────────────────────────

    def _sample_perf_if_due(self, last_sample: float):
        """如果距上一次采样已超过 PERF_SAMPLE_INTERVAL，则采样一次。返回新的 last_sample。"""
        now = time.time()
        if now - last_sample >= PERF_SAMPLE_INTERVAL:
            cpu, mem = sample_cpu_mem()
            self._cpu_samples.append(cpu)
            self._mem_samples.append(mem)
            self.get_logger().info(f"  性能采样 [{len(self._cpu_samples)}]: CPU {cpu:.1f}%  MEM {mem:.1f}%")
            return now
        return last_sample

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
        # 自动启动建图+导航（启动失败，改为手动启动）
        # self._start_navigation()

        # 等待定位
        self.get_logger().info(f"等待定位数据 (topic: {self._pose_topic})...")
        deadline = time.time() + 20.0
        while self._active_pose_topic is None and rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._active_pose_topic is None:
            self.get_logger().error(f"20s 内无定位数据！检查: {self._pose_topic}")
            self._stop_navigation()
            return

        cp = self._get_current_pose()
        if cp is not None:
            x_p, y_p, yaw_deg_p = map_to_paper(cp[0], cp[1], cp[2])
            self.get_logger().info(f"初始位姿 map:({cp[0]:.3f},{cp[1]:.3f},{math.degrees(cp[2]):.1f}°)  "
                                   f"论文:({x_p:.3f},{y_p:.3f},{yaw_deg_p:.1f}°)")

        self.get_logger().info("等待 Nav2 action server...")
        if not self._action_client.wait_for_server(timeout_sec=60):
            self.get_logger().error("Nav2 未就绪")
            self._stop_navigation()
            return

        # ── 导航三角形 1→3→4→1 ──────────────────────────────────────
        for idx in range(len(GOALS_MAP)):
            gx, gy, gyaw = GOALS_MAP[idx]
            label = PAPER_LABELS[idx]
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"航点 {label} — map:({gx:.2f},{gy:.2f},{math.degrees(gyaw):.0f}°)")

            if idx == 0:
                ok = self._send_goal_and_wait(gx, gy, gyaw, record_traj=False)
            else:
                ok = self._send_goal_and_wait(gx, gy, gyaw, record_traj=True)

            if not ok:
                self.get_logger().error(f"航点 {label} 导航失败")
                self._stop_navigation()
                return

            # 等待机器人停稳再记录位姿（Nav2 到达≠停止旋转）
            self.get_logger().info(f"  停稳等待 3s...")
            settle_deadline = time.monotonic() + 3.0
            while time.monotonic() < settle_deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

            cp = self._get_current_pose()
            if cp is None:
                self.get_logger().error(f"航点 {label} 到达后定位丢失")
                self._stop_navigation()
                return
            self._slam_poses.append(cp)
            x_p, y_p, yaw_deg_p = map_to_paper(cp[0], cp[1], cp[2])
            self.get_logger().info(f"  到达点{label} SLAM(map):({cp[0]:.3f},{cp[1]:.3f},{math.degrees(cp[2]):.1f}°)  "
                                   f"SLAM(论文):({x_p:.3f},{y_p:.3f},{yaw_deg_p:.1f}°)")

            if idx > 0:
                self._compute_segment_cte(idx - 1)

            if idx in GT_WAYPOINTS:
                self._collect_static_at_waypoint(label)

                # 用 print 确保提示在终端可见（ros2 logger 可能缓冲）
                print(f"\n  >>> 点{label} 静态采集完成，按 Enter 开始输入地面真值...")
                input()

                exp_x, exp_y, exp_yaw = GT_WAYPOINTS[idx]
                print(f"  点{label} 理论论文坐标: ({exp_x}, {exp_y}, {exp_yaw}°)")
                print(f"  输入偏差 (dx dy yaw_actual°)，如: 0.02 -0.04 176")
                print("  GT (dx dy yaw_actual): ", end="", flush=True)
                gt_str = sys.stdin.readline().strip()
                try:
                    parts = gt_str.split()
                    dx = float(parts[0])
                    dy = float(parts[1])
                    yaw_actual = float(parts[2]) if len(parts) > 2 else exp_yaw
                except (ValueError, IndexError):
                    self.get_logger().error(f"输入格式错误: '{gt_str}'")
                    self._stop_navigation()
                    return

                gt_x_p = exp_x + dx   # 用户直接测量车中心点（与 SLAM base_footprint 一致）
                gt_y_p = exp_y + dy
                gt_yaw_deg_p = yaw_actual
                self.get_logger().info(
                    f"  实际论文坐标: ({gt_x_p:.3f}, {gt_y_p:.3f}, {gt_yaw_deg_p:.1f}°)")

                self._gt_poses_paper.append((gt_x_p, gt_y_p, gt_yaw_deg_p))
                gt_map = paper_to_map(gt_x_p, gt_y_p, gt_yaw_deg_p)
                self._gt_poses_map.append(gt_map)

                # ── 记录航点原始数据（含用户 GT 输入，可复核）──
                self._waypoint_rows.append({
                    "label": label,
                    "slam_x": cp[0], "slam_y": cp[1], "slam_yaw_deg": math.degrees(cp[2]),
                    "gt_input_dx": dx, "gt_input_dy": dy, "gt_input_yaw_deg": yaw_actual,
                    "gt_paper_x": gt_x_p, "gt_paper_y": gt_y_p, "gt_paper_yaw_deg": gt_yaw_deg_p,
                    "gt_map_x": gt_map[0], "gt_map_y": gt_map[1], "gt_map_yaw_deg": math.degrees(gt_map[2]),
                })

                # 仅 1→3 段 (idx==1, 到达点3): 手动输入碰撞
                if idx == 1:
                    print("  >>> 1→3 是否碰撞? (0=无碰撞 1=碰撞): ", end="", flush=True)
                    coll_str = sys.stdin.readline().strip()
                    self._collisions.append(int(coll_str) if coll_str in ("0", "1") else 0)
                else:
                    self._collisions.append(0)

        # ── 全部航点完成 ───────────────────────────────────────────
        self._success = True
        self._save_waypoints()
        self._compute_and_log_rpe()
        self._write_results()
        self._save_perf_results()
        self.get_logger().info("所有测试完成 ✓")

    def _save_perf_results(self):
        """仅在成功时保存性能采样（均值 + 标准差）。"""
        if not self._cpu_samples:
            return
        n = len(self._cpu_samples)
        avg_cpu = sum(self._cpu_samples) / n
        avg_mem = sum(self._mem_samples) / n
        std_cpu = (sum((x - avg_cpu)**2 for x in self._cpu_samples) / n) ** 0.5 if n > 1 else 0.0
        std_mem = (sum((x - avg_mem)**2 for x in self._mem_samples) / n) ** 0.5 if n > 1 else 0.0
        self.get_logger().info(
            f"性能: CPU {avg_cpu:.1f}±{std_cpu:.1f}%  MEM {avg_mem:.1f}±{std_mem:.1f}% (n={n})")

        # 原始采样 → logs/
        perf_path = os.path.join(PERF_LOG_DIR, f"{self._sensor}_perf.csv")
        try:
            save_perf_samples(perf_path, self._cpu_samples, self._mem_samples)
            self.get_logger().info(f"原始采样已保存: {perf_path}")
        except PermissionError:
            self.get_logger().error(f"无写入权限: {perf_path}")

        # 汇总 → tools/results/
        try:
            headers = ["timestamp", "sensor",
                       "cpu_mean_pct", "cpu_std_pct", "mem_mean_pct", "mem_std_pct", "num_samples"]
            logger = AppendingCSVLogger(CSV_PERF_PATH, headers)
            row = [f"{time.time():.3f}", self._sensor,
                   f"{avg_cpu:.1f}", f"{std_cpu:.1f}", f"{avg_mem:.1f}", f"{std_mem:.1f}", str(n)]
            logger.add_row(row)
            logger.close()
            self.get_logger().info(f"性能汇总已保存: {CSV_PERF_PATH}")
        except PermissionError:
            self.get_logger().error(f"无写入权限: {CSV_PERF_PATH}")

    def _save_trajectory(self, seg_idx, traj):
        """保存逐段导航原始轨迹到 logs/trajectory/。"""
        if not traj:
            return
        s, e = SEGMENT_NAMES[seg_idx]
        ts = int(time.time())
        path = os.path.join(TRAJ_LOG_DIR, f"traj_{self._sensor}_seg{s}-{e}_{ts}.csv")
        try:
            with open(path, "w") as f:
                f.write("t,x,y,yaw\n")
                for t, px, py, pyaw in traj:
                    f.write(f"{t:.3f},{px:.6f},{py:.6f},{pyaw:.6f}\n")
            self.get_logger().info(f"  轨迹已保存: {path}")
        except PermissionError:
            self.get_logger().warn(f"轨迹保存失败(权限): {path}")

    def _save_waypoints(self):
        """保存航点原始数据（含 SLAM 位姿 + 用户 GT 输入）到 logs/waypoints/。"""
        if not self._waypoint_rows:
            return
        ts = int(time.time())
        path = os.path.join(WP_LOG_DIR, f"waypoints_{self._sensor}_{ts}.csv")
        headers = ["label", "slam_x", "slam_y", "slam_yaw_deg",
                   "gt_input_dx", "gt_input_dy", "gt_input_yaw_deg",
                   "gt_paper_x", "gt_paper_y", "gt_paper_yaw_deg",
                   "gt_map_x", "gt_map_y", "gt_map_yaw_deg"]
        try:
            with open(path, "w") as f:
                f.write(",".join(headers) + "\n")
                for w in self._waypoint_rows:
                    row = [str(w[h]) for h in headers]
                    f.write(",".join(row) + "\n")
            self.get_logger().info(f"航点原始数据已保存: {path}")
        except PermissionError:
            self.get_logger().warn(f"航点保存失败(权限): {path}")

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
        last_perf_sample = start_time
        traj = []

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            # 性能采样
            last_perf_sample = self._sample_perf_if_due(last_perf_sample)

            cp = self._get_current_pose()
            if cp is not None:
                last_localized_time = now
                if record_traj:
                    if not traj or now - traj[-1][0] >= 1.0 / CTE_SAMPLE_HZ:
                        traj.append((now, cp[0], cp[1], cp[2]))
            elif now - last_localized_time > LOCALIZATION_LOST_TIMEOUT:
                self.get_logger().error(f"定位丢失 {LOCALIZATION_LOST_TIMEOUT:.0f}s")
                goal_handle.cancel_goal_async()
                return False

            if now - start_time > GOAL_TIMEOUT_SEC:
                self.get_logger().error(f"目标超时 {GOAL_TIMEOUT_SEC:.0f}s")
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        if result is None:
            self.get_logger().error("目标执行失败")
            return False

        if record_traj and traj:
            self._segment_trajectories.append(traj)
            self._save_trajectory(len(self._segment_trajectories) - 1, traj)
            self.get_logger().info(f"  轨迹采样: {len(traj)} 点")

        self.get_logger().info("  目标已到达 ✓")
        return True

    # ══════════════════════════════════════════════════════════════════════
    # CTE 计算（3 段: 1→3, 3→4, 4→1）
    # ══════════════════════════════════════════════════════════════════════

    def _compute_segment_cte(self, seg_idx):
        if seg_idx >= len(self._segment_trajectories):
            return

        traj = self._segment_trajectories[seg_idx]
        if len(traj) < 2:
            self._segment_cte.append((float("nan"), float("nan")))
            return

        sx, sy, _ = GOALS_MAP[seg_idx]
        ex, ey, _ = GOALS_MAP[seg_idx + 1]
        seg_dx = ex - sx
        seg_dy = ey - sy
        seg_len = math.sqrt(seg_dx ** 2 + seg_dy ** 2)

        if seg_len < 1e-6:
            errors = [math.sqrt((p[1] - sx) ** 2 + (p[2] - sy) ** 2) for p in traj]
        else:
            ux = seg_dx / seg_len
            uy = seg_dy / seg_len
            errors = []
            for _, px, py, _ in traj:
                proj = (px - sx) * ux + (py - sy) * uy
                cx = sx + proj * ux
                cy = sy + proj * uy
                errors.append(math.sqrt((px - cx) ** 2 + (py - cy) ** 2))

        cte_rmse = math.sqrt(sum(e ** 2 for e in errors) / len(errors))
        cte_max = max(errors)
        self._segment_cte.append((cte_rmse, cte_max))
        s, e = SEGMENT_NAMES[seg_idx]
        self.get_logger().info(f"  CTE {s}→{e}: RMSE={cte_rmse:.4f}m  MAX={cte_max:.4f}m")

    # ══════════════════════════════════════════════════════════════════════
    # 静态稳定性（在航点暂停时采集）
    # ══════════════════════════════════════════════════════════════════════

    def _collect_static_at_waypoint(self, label):
        self.get_logger().info(f"  >>> 点{label} 静态稳定性采集 {STATIC_DURATION_SEC:.0f}s...")

        # 稳定等待 3s，车辆停止
        settle_deadline = time.monotonic() + 3.0
        while time.monotonic() < settle_deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # 回调驱动采样：每次 /localization_pose 或 /odom 更新时自动记录
        self._static_samples = []
        self._static_collecting = True
        start_mono = time.monotonic()
        while time.monotonic() - start_mono < STATIC_DURATION_SEC and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        self._static_collecting = False
        samples = self._static_samples

        if len(samples) < 5:
            self.get_logger().warn(f"  点{label} 静态样本不足 (n={len(samples)})")
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
        try:
            csv_logger = CSVLogger(
                RAW_LOG_DIR, f"static_{self._sensor}_pt{label}",
                ["x", "y", "yaw"])
            for x, y, yaw in samples:
                csv_logger.add_row([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}"])
            csv_logger.close()
        except PermissionError:
            self.get_logger().warn(f"原始静态数据保存失败（权限不足），跳过: {RAW_LOG_DIR}/")

    def _save_static_result(self, label, n_samples, rmse_pos, rmse_yaw, max_pos, max_yaw):
        """暂存静态结果到内存，全测试成功后才写入 CSV。"""
        self._static_results.append((label, n_samples, rmse_pos, rmse_yaw, max_pos, max_yaw))

    def _write_static_csv(self):
        """仅在全部成功时将缓存的静态结果写入 CSV。"""
        if not self._static_results:
            return
        headers = ["timestamp", "sensor", "waypoint", "n_samples",
                   "rmse_pos_m", "rmse_yaw_deg", "max_pos_m", "max_yaw_deg"]
        logger = AppendingCSVLogger(CSV_STATIC_PATH, headers)
        for label, n, rpos, ryaw, mpos, myaw in self._static_results:
            logger.add_row([
                f"{time.time():.3f}", self._sensor, str(label), str(n),
                f"{rpos:.6f}", f"{math.degrees(ryaw):.6f}",
                f"{mpos:.6f}", f"{math.degrees(myaw):.6f}",
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
        self.get_logger().info("            三角形路径 RPE 汇总")
        self.get_logger().info("=" * 60)

        # slam: [pt1_start, pt3, pt4, pt1_end]; gt: [pt3, pt4, pt1]
        sc = [self._slam_poses[1], self._slam_poses[2], self._slam_poses[3]]
        paper_labels = [3, 4, 1]
        self.get_logger().info(f"{'点':>4} {'SLAM_x':>8} {'SLAM_y':>8} {'SLAM_yaw°':>8}  "
                               f"{'GT_x':>8} {'GT_y':>8} {'GT_yaw°':>8}")
        for i in range(N):
            sx, sy, syaw = sc[i]
            gx, gy, gyaw = self._gt_poses_map[i]
            self.get_logger().info(f"{paper_labels[i]:>4} {sx:>8.3f} {sy:>8.3f} {math.degrees(syaw):>8.1f}  "
                                   f"{gx:>8.3f} {gy:>8.3f} {math.degrees(gyaw):>8.1f}")

        self._save_rpe_results()

    def _save_rpe_results(self):
        """三角形 3 段 RPE（含标准差、相对误差、闭合误差）。"""
        N = len(self._gt_poses_map)
        if N < 2:
            return

        gt_map = self._gt_poses_map  # [(pt3), (pt4), (pt1)]
        sc = [self._slam_poses[1], self._slam_poses[2], self._slam_poses[3]]

        # ── 每段绝对误差 ──
        seg_pos, seg_yaw = [], []
        seg_gt_len = []  # GT 线段长度 (m)
        for i in range(N):
            j = (i + 1) % N
            sx0, sy0, syaw0 = sc[i]; sx1, sy1, syaw1 = sc[j]
            gx0, gy0, gyaw0 = gt_map[i]; gx1, gy1, gyaw1 = gt_map[j]
            e_pos = math.hypot((sx1 - sx0) - (gx1 - gx0), (sy1 - sy0) - (gy1 - gy0))
            e_yaw = abs(normalize_angle(normalize_angle(syaw1 - syaw0) - normalize_angle(gyaw1 - gyaw0)))
            seg_pos.append(e_pos)
            seg_yaw.append(e_yaw)
            seg_gt_len.append(math.hypot(gx1 - gx0, gy1 - gy0))

        n_seg = len(seg_pos)
        mean_pos = sum(seg_pos) / n_seg
        mean_yaw = sum(seg_yaw) / n_seg
        std_pos = (sum((x - mean_pos)**2 for x in seg_pos) / n_seg) ** 0.5 if n_seg > 1 else 0.0
        std_yaw = (sum((x - mean_yaw)**2 for x in seg_yaw) / n_seg) ** 0.5 if n_seg > 1 else 0.0

        # ── 每段相对误差 (% of 实际轨迹路程) ──
        # _segment_trajectories: [1→3, 3→4, 4→1]; seg_pos: [3→4, 4→1, 1→3]
        seg_traj_len = [0.0, 0.0, 0.0]
        if len(self._segment_trajectories) >= n_seg:
            raw = []
            for traj in self._segment_trajectories[:n_seg]:
                pl = 0.0
                for k in range(1, len(traj)):
                    pl += math.hypot(traj[k][1] - traj[k-1][1], traj[k][2] - traj[k-1][2])
                raw.append(pl)
            # raw: [1→3, 3→4, 4→1] → 重排为 [3→4, 4→1, 1→3]
            seg_traj_len = [raw[1], raw[2], raw[0]]
        else:
            seg_traj_len = seg_gt_len
        seg_rel_pos = [seg_pos[i] / seg_traj_len[i] * 100.0 for i in range(n_seg)]
        mean_rel = sum(seg_rel_pos) / n_seg
        std_rel = (sum((x - mean_rel)**2 for x in seg_rel_pos) / n_seg) ** 0.5 if n_seg > 1 else 0.0

        # ── 总路程（从实际轨迹计算，因为 1→3 有避障曲线）──
        total_path_len = 0.0
        if len(self._segment_trajectories) >= 3:
            for traj in self._segment_trajectories[:3]:
                for k in range(1, len(traj)):
                    total_path_len += math.hypot(traj[k][1] - traj[k-1][1],
                                                 traj[k][2] - traj[k-1][2])

        self.get_logger().info(
            f"\nRPE 汇总: 位置 {mean_pos:.4f}±{std_pos:.4f} m  航向 {math.degrees(mean_yaw):.2f}±{math.degrees(std_yaw):.2f}°\n"
            f"  总路程 {total_path_len:.3f} m")

        headers = ["timestamp", "sensor",
                   "seg_3to4_pos_m", "seg_3to4_yaw_deg", "seg_3to4_rel_pct",
                   "seg_4to1_pos_m", "seg_4to1_yaw_deg", "seg_4to1_rel_pct",
                   "seg_1to3_pos_m", "seg_1to3_yaw_deg", "seg_1to3_rel_pct",
                   "mean_pos_m", "std_pos_m", "mean_yaw_deg", "std_yaw_deg",
                   "mean_rel_pct", "std_rel_pct",
                   "closure_rel_pct",
                   "total_path_len_m",
                   "collision_1to3"]
        logger = AppendingCSVLogger(CSV_RPE_PATH, headers)
        row = [f"{time.time():.3f}", self._sensor]
        for i in range(3):
            row.append(f"{seg_pos[i]:.4f}")
            row.append(f"{math.degrees(seg_yaw[i]):.4f}")
            row.append(f"{seg_rel_pos[i]:.2f}")
        row.append(f"{mean_pos:.4f}")
        row.append(f"{std_pos:.4f}")
        row.append(f"{math.degrees(mean_yaw):.4f}")
        row.append(f"{math.degrees(std_yaw):.4f}")
        row.append(f"{mean_rel:.2f}")
        row.append(f"{std_rel:.2f}")
        closure_rel = seg_pos[1] / total_path_len * 100.0 if total_path_len > 0 else 0.0
        row.append(f"{closure_rel:.4f}")
        row.append(f"{total_path_len:.3f}")
        collision_val = self._collisions[0] if self._collisions else 0
        row.append(str(collision_val))
        logger.add_row(row)
        logger.close()

    # ══════════════════════════════════════════════════════════════════════
    # 结果写入
    # ══════════════════════════════════════════════════════════════════════

    def _write_results(self):
        """仅在全部成功时写入 CTE (3段) 和静态结果。"""
        try:
            if self._segment_cte:
                headers = ["timestamp", "sensor",
                           "cte_1to3_rmse_m", "cte_1to3_max_m",
                           "cte_3to4_rmse_m", "cte_3to4_max_m",
                           "cte_4to1_rmse_m", "cte_4to1_max_m"]
                logger = AppendingCSVLogger(CSV_CTE_PATH, headers)
                row = [f"{time.time():.3f}", self._sensor]
                for i in range(3):
                    if i < len(self._segment_cte):
                        row.append(f"{self._segment_cte[i][0]:.4f}")
                        row.append(f"{self._segment_cte[i][1]:.4f}")
                    else:
                        row.extend(["", ""])
                logger.add_row(row)
                logger.close()

            self._write_static_csv()

            self.get_logger().info(f"\n论文数据已保存到 {RESULTS_DIR}/")
            self.get_logger().info(f"  RPE:    {CSV_RPE_PATH}")
            self.get_logger().info(f"  CTE:    {CSV_CTE_PATH}")
            self.get_logger().info(f"  Static: {CSV_STATIC_PATH}")
            self.get_logger().info(f"  Perf:   {CSV_PERF_PATH}")
            self.get_logger().info(f"原始轨迹 -> {RAW_LOG_DIR}/")
        except PermissionError as e:
            self.get_logger().error(f"写入权限不足: {e}")
            self.get_logger().error(f"请执行: chmod -R u+w {RESULTS_DIR}/ {RAW_LOG_DIR}/ {PERF_LOG_DIR}/")


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
        node._stop_navigation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
