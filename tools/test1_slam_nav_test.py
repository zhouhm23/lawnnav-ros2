#!/usr/bin/env python3
"""
test1_slam_nav_test.py — SLAM与导航综合测试（闭合路径RPE + 静态定位稳定性）。

用法:
    python3 test1_slam_nav_test.py --mode rpe      # 仅闭合路径RPE
    python3 test1_slam_nav_test.py --mode static   # 仅静态定位稳定性
    python3 test1_slam_nav_test.py --mode all      # 全部（默认）

坐标系说明:
    ┌─────────────────────┬──────────────────────────────┐
    │ 论文坐标系           │ Nav2 / map 坐标系             │
    ├─────────────────────┼──────────────────────────────┤
    │ y⁺ = 车初始朝向      │ x⁺ = 车初始朝向               │
    │ x⁺ = 车体右侧        │ y⁺ = 车体左侧                 │
    │ 起点1 = (0.4, 0)     │ 起点1 = (0, 0)               │
    ├─────────────────────┼──────────────────────────────┤
    │ 转换: x_map = y_paper                              │
    │       y_map = -x_paper + 0.4                        │
    │       ψ_map = ψ_paper - π/2                         │
    └─────────────────────┴──────────────────────────────┘

    RPE 使用相对位移差 Δ_slam - Δ_gt 计算，全局坐标系旋转/平移在相减时抵消，
    因此 RPE 结果不受论文坐标系与 map 坐标系不对齐的影响。

测试流程:
  --mode rpe:
    1. 等待 Nav2 action server 和 localization
    2. 按 2→3→4→1 顺序导航闭合路径（map 坐标系目标点）
    3. 每个目标点到达后暂停，等待用户在终端输入论文坐标系下的地面真值
    4. 脚本自动将论文坐标转换为 map 坐标
    5. 计算逐段 RPE（4段）和端到端闭合误差
    6. 终端打印对照表，结果写入 rpe_results.csv

  --mode static:
    1. 等待 localization 就绪
    2. 以 5Hz 采样位姿 60s，写入 pose_log_*.csv
    3. 以第一帧为参考原点，计算 RMSE 抖动和最大漂移量
    4. 结果写入 static_stability_results.csv

  --mode all:
    先执行 rpe 模式，完成后自动执行 static 模式。
"""

import argparse
import math
import sys
import time

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

GOAL_TIMEOUT_SEC = 120.0          # 单目标导航超时（秒）
LOCALIZATION_LOST_TIMEOUT = 3.0   # 定位丢失容忍（秒）
STATIC_DURATION_SEC = 60.0        # 静止记录时长（秒）
STATIC_RATE_HZ = 5.0              # 静止记录采样频率（Hz）

# 多源定位 topic 候选列表（按优先级排序，首个有数据的生效）
POSE_TOPIC_CANDIDATES = [
    "/rtabmap/localization_pose",   # RTAB-Map SLAM 校正位姿（优先）
    "/localization_pose",           # remapped 或 robot_localization
]

# Nav2 导航目标点（map 坐标系，车头=x⁺，左侧=y⁺）
# 路径: 点2 → 点3 → 点4 → 点1（闭合矩形）
GOALS_MAP = [
    # (x, y, yaw)    论文对应点
    (1.8,  0.0, -math.pi / 2.0),   # 论文点2: (0.4, 1.8)
    (1.8, -1.0, -math.pi),         # 论文点3: (1.4, 1.8)
    (0.0, -1.0,  math.pi / 2.0),   # 论文点4: (1.4, 0)
    (0.0,  0.0,  0.0),             # 论文点1: (0.4, 0)
]

# CSV 输出路径
CSV_RPE_PATH = "/home/ubuntu/ros2_ws/src/tools/rpe_results.csv"
CSV_STATIC_PATH = "/home/ubuntu/ros2_ws/src/tools/static_stability_results.csv"
POSE_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/pose"


# ═══════════════════════════════════════════════════════════════════════════
# 坐标系转换工具（paper ↔ map）
# ═══════════════════════════════════════════════════════════════════════════

def paper_to_map(x_p: float, y_p: float, yaw_deg_p: float = 0.0):
    """将论文坐标系位姿转换为 map 坐标系。

    论文: y⁺=车头方向, x⁺=右侧, 起点1=(0.4,0)
    map:  x⁺=车头方向, y⁺=左侧, 起点1=(0,0)
    """
    x_m = y_p
    y_m = -x_p + 0.4
    yaw_m = normalize_angle(math.radians(yaw_deg_p) - math.pi / 2.0)
    return (x_m, y_m, yaw_m)


def map_to_paper(x_m: float, y_m: float, yaw_m: float = 0.0):
    """将 map 坐标系位姿转换为论文坐标系。"""
    x_p = -y_m + 0.4
    y_p = x_m
    yaw_deg_p = math.degrees(normalize_angle(yaw_m + math.pi / 2.0))
    return (x_p, y_p, yaw_deg_p)


# ═══════════════════════════════════════════════════════════════════════════
# SlamNavTest 节点
# ═══════════════════════════════════════════════════════════════════════════

class SlamNavTest(Node):
    def __init__(self, mode: str = "all"):
        node_name = "slam_nav_test"
        super().__init__(node_name)

        self._mode = mode
        self._do_rpe = mode in ("rpe", "all")
        self._do_static = mode in ("static", "all")

        # ── 多源定位订阅 ────────────────────────────────────────────────
        # 同时订阅多个候选 topic，自动选用首个有数据的
        self._pose_cache = {}           # topic → latest PoseWithCovarianceStamped
        self._active_pose_topic = None  # 生效的 topic 名
        for topic in POSE_TOPIC_CANDIDATES:
            self.create_subscription(
                PoseWithCovarianceStamped,
                topic,
                self._make_pose_callback(topic),
                10,
            )
            self._pose_cache[topic] = None

        # ── Action client（仅 rpe 模式需要）──────────────────────────────
        self._action_client = None
        if self._do_rpe:
            self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── cmd_vel publisher（预留，暂不旋转）────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── 数据存储 ─────────────────────────────────────────────────────
        self._slam_poses = []   # 每个目标点到达时的 SLAM 估计位姿 (x,y,yaw)
        self._gt_poses_paper = []  # 用户输入的地面真值（论文坐标系）(x,y,yaw_deg)
        self._gt_poses_map = []    # 转换后的地面真值（map 坐标系）(x,y,yaw)

    # ── 回调 ─────────────────────────────────────────────────────────────

    def _make_pose_callback(self, topic: str):
        """为每个候选 topic 生成独立回调，自动激活首个有数据的源。"""
        def cb(msg):
            self._pose_cache[topic] = msg
            if self._active_pose_topic is None and msg is not None:
                self._active_pose_topic = topic
                self.get_logger().info(f"定位源已激活: {topic}")
        return cb

    # ── 位姿辅助 ─────────────────────────────────────────────────────────

    def _get_latest_pose_msg(self):
        """返回当前激活 topic 的最新消息。"""
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

    def _get_current_stamp(self):
        """返回当前激活 topic 最新消息的时间戳 (sec, nanosec)。"""
        msg = self._get_latest_pose_msg()
        if msg is None:
            return (0, 0)
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    # ══════════════════════════════════════════════════════════════════════
    # 主入口
    # ══════════════════════════════════════════════════════════════════════

    def run(self):
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
            x_p, y_p, yaw_deg_p = map_to_paper(cp[0], cp[1], cp[2])
            self.get_logger().info(
                f"初始位姿 | map: ({cp[0]:.3f}, {cp[1]:.3f}, {math.degrees(cp[2]):.1f}°)  "
                f"论文: ({x_p:.3f}, {y_p:.3f}, {yaw_deg_p:.1f}°)"
            )

        # ── 执行 RPE 测试 ────────────────────────────────────────────────
        if self._do_rpe:
            self._run_rpe_test()

        # ── 执行静态稳定性测试 ──────────────────────────────────────────
        if self._do_static:
            self._run_static_test()

        self.get_logger().info("所有测试完成 ✓")

    # ══════════════════════════════════════════════════════════════════════
    # RPE 测试
    # ══════════════════════════════════════════════════════════════════════

    def _run_rpe_test(self):
        self.get_logger().info("========== RPE 测试开始 ==========")

        # 等待 Nav2
        self.get_logger().info("等待 Nav2 action server...")
        self._action_client.wait_for_server()

        # 导航并逐点收集 GT
        for idx, (gx, gy, gyaw) in enumerate(GOALS_MAP, start=1):
            paper_idx = [2, 3, 4, 1][idx - 1]  # 论文编号
            self.get_logger().info(
                f"--- 目标 {idx}/4 (论文点{paper_idx}) ---"
                f" map: ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f}°)")

            ok = self._send_goal_and_wait(gx, gy, gyaw)
            if not ok:
                self.get_logger().error(f"目标 {idx} 失败，RPE 测试中止。")
                return

            # 记录 SLAM 估计位姿
            cp = self._get_current_pose()
            if cp is None:
                self.get_logger().error("到达目标点后定位丢失，RPE 测试中止。")
                return
            self._slam_poses.append(cp)

            # 打印 SLAM 位姿（两种坐标系）
            x_p, y_p, yaw_deg_p = map_to_paper(cp[0], cp[1], cp[2])
            self.get_logger().info(
                f"  到达点{paper_idx} — "
                f"SLAM(map): ({cp[0]:.3f}, {cp[1]:.3f}, {math.degrees(cp[2]):.1f}°)  "
                f"SLAM(论文): ({x_p:.3f}, {y_p:.3f}, {yaw_deg_p:.1f}°)"
            )

            # 等待用户输入论文坐标系下的地面真值
            self.get_logger().info(
                f"  >>> 请输入论文点{paper_idx}的地面真值 (x_paper y_paper yaw_deg_paper):")
            gt_str = input("  GT (论文坐标系 x y yaw_deg): ").strip()
            try:
                parts = gt_str.split()
                gt_x_p = float(parts[0])
                gt_y_p = float(parts[1])
                gt_yaw_deg_p = float(parts[2]) if len(parts) > 2 else 0.0
            except (ValueError, IndexError):
                self.get_logger().error(f"输入格式错误: '{gt_str}'，预期 'x y yaw_deg'")
                return

            self._gt_poses_paper.append((gt_x_p, gt_y_p, gt_yaw_deg_p))
            gt_map = paper_to_map(gt_x_p, gt_y_p, gt_yaw_deg_p)
            self._gt_poses_map.append(gt_map)
            self.get_logger().info(
                f"  GT(论文): ({gt_x_p:.3f}, {gt_y_p:.3f}, {gt_yaw_deg_p:.1f}°)  →  "
                f"GT(map): ({gt_map[0]:.3f}, {gt_map[1]:.3f}, {math.degrees(gt_map[2]):.1f}°)"
            )

        # 计算 RPE
        self._compute_and_log_rpe()

    # ── 导航到目标点 ────────────────────────────────────────────────────

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
                return False

            if now - start_time > GOAL_TIMEOUT_SEC:
                self.get_logger().error(
                    f"目标超时 {GOAL_TIMEOUT_SEC:.0f}s — 取消")
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        if result is None:
            self.get_logger().error("目标执行失败")
            return False

        self.get_logger().info("  目标已到达 ✓")
        return True

    # ── RPE 计算与输出 ──────────────────────────────────────────────────

    def _compute_and_log_rpe(self):
        """计算逐段 RPE 和端到端闭合误差，终端打印并写入 CSV。"""
        N = len(self._slam_poses)
        if N < 2:
            self.get_logger().error("样本不足，无法计算 RPE")
            return

        self.get_logger().info("\n" + "=" * 72)
        self.get_logger().info("              闭合路径相对位姿误差 (RPE)")
        self.get_logger().info("=" * 72)

        # ── 位姿对照表 ──────────────────────────────────────────────────
        self.get_logger().info(
            f"{'点':>4} {'SLAM_x_map':>10} {'SLAM_y_map':>10} {'SLAM_yaw°':>10}  "
            f"{'GT_x_map':>10} {'GT_y_map':>10} {'GT_yaw°':>10}  "
            f"{'GT_x_paper':>10} {'GT_y_paper':>10} {'GT_yaw°_paper':>10}")
        self.get_logger().info("-" * 72)
        for i in range(N):
            sx, sy, syaw = self._slam_poses[i]
            gx, gy, gyaw = self._gt_poses_map[i]
            px, py, pyaw_deg = self._gt_poses_paper[i]
            self.get_logger().info(
                f"{i+1:>4} {sx:>10.3f} {sy:>10.3f} {math.degrees(syaw):>10.1f}  "
                f"{gx:>10.3f} {gy:>10.3f} {math.degrees(gyaw):>10.1f}  "
                f"{px:>10.3f} {py:>10.3f} {pyaw_deg:>10.1f}")

        # ── 逐段 RPE ────────────────────────────────────────────────────
        self.get_logger().info("\n--- 逐段相对定位误差 (Segment-wise RPE) ---")
        self.get_logger().info(
            f"{'段':>6} {'Δx_slam':>10} {'Δy_slam':>10} {'Δψ_slam°':>10}  "
            f"{'Δx_gt':>10} {'Δy_gt':>10} {'Δψ_gt°':>10}  "
            f"{'e_pos(m)':>10} {'e_ψ(°)':>10}")

        seg_errors_pos = []
        seg_errors_yaw = []

        for i in range(N):   # N=4 段: 2→3, 3→4, 4→1, 1→2（最后一段 wrap 到索引 0）
            j = (i + 1) % N  # wrap-around 闭合
            sx0, sy0, syaw0 = self._slam_poses[i]
            sx1, sy1, syaw1 = self._slam_poses[j]
            gx0, gy0, gyaw0 = self._gt_poses_map[i]
            gx1, gy1, gyaw1 = self._gt_poses_map[j]

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

            # 论文编号
            paper_order = [2, 3, 4, 1]
            paper_from = paper_order[i]
            paper_to = paper_order[j]
            self.get_logger().info(
                f"{paper_from}→{paper_to}:  "
                f"{ds_x:>10.3f} {ds_y:>10.3f} {math.degrees(ds_yaw):>10.1f}  "
                f"{dg_x:>10.3f} {dg_y:>10.3f} {math.degrees(dg_yaw):>10.1f}  "
                f"{e_pos:>10.4f} {math.degrees(e_yaw):>10.2f}")

        # ── 端到端闭合误差 ──────────────────────────────────────────────
        sx0, sy0, syaw0 = self._slam_poses[0]
        sxN, syN, syawN = self._slam_poses[-1]  # 最后到达的 SLAM 位姿（回到起点附近）
        gx0, gy0, gyaw0 = self._gt_poses_map[0]
        gxN, gyN, gyawN = self._gt_poses_map[-1]

        ds_loop_x = sxN - sx0
        ds_loop_y = syN - sy0
        ds_loop_yaw = normalize_angle(syawN - syaw0)

        dg_loop_x = gxN - gx0
        dg_loop_y = gyN - gy0
        dg_loop_yaw = normalize_angle(gyawN - gyaw0)

        e_loop_pos = math.sqrt((ds_loop_x - dg_loop_x) ** 2
                               + (ds_loop_y - dg_loop_y) ** 2)
        e_loop_yaw = abs(normalize_angle(ds_loop_yaw - dg_loop_yaw))

        self.get_logger().info("\n--- 端到端闭合误差 (End-to-end Loop Closure) ---")
        self.get_logger().info(
            f"  SLAM 位移: Δx={ds_loop_x:.4f} Δy={ds_loop_y:.4f} Δψ={math.degrees(ds_loop_yaw):.2f}°")
        self.get_logger().info(
            f"  GT   位移: Δx={dg_loop_x:.4f} Δy={dg_loop_y:.4f} Δψ={math.degrees(dg_loop_yaw):.2f}°")
        self.get_logger().info(
            f"  闭合误差: e_pos={e_loop_pos:.4f} m  e_ψ={math.degrees(e_loop_yaw):.4f}°")
        self.get_logger().info("=" * 72 + "\n")

        # ── 写入 CSV ────────────────────────────────────────────────────
        self._write_rpe_csv(seg_errors_pos, seg_errors_yaw,
                            e_loop_pos, e_loop_yaw,
                            ds_loop_x, ds_loop_y, ds_loop_yaw,
                            dg_loop_x, dg_loop_y, dg_loop_yaw)

    def _write_rpe_csv(self, seg_errors_pos, seg_errors_yaw,
                       e_loop_pos, e_loop_yaw,
                       ds_loop_x, ds_loop_y, ds_loop_yaw,
                       dg_loop_x, dg_loop_y, dg_loop_yaw):
        """写入 RPE 结果到 CSV（AppendingCSVLogger）。"""
        N = len(self._slam_poses)

        # ── 构建 GT raw 字符串 ──────────────────────────────────────────
        paper_order = [2, 3, 4, 1]
        gt_raw_parts = []
        for i in range(N):
            px, py, pyaw_deg = self._gt_poses_paper[i]
            gt_raw_parts.append(f"Pt{paper_order[i]}:({px:.3f},{py:.3f},{pyaw_deg:.1f})")
        gt_raw_str = "; ".join(gt_raw_parts)

        headers = [
            "timestamp",
            # SLAM 位姿 (map 坐标系) — 按论文编号列名
            *[f"slam_pt{pn}_x" for pn in paper_order],
            *[f"slam_pt{pn}_y" for pn in paper_order],
            *[f"slam_pt{pn}_yaw_deg" for pn in paper_order],
            # GT 位姿 (论文坐标系，原始输入)
            *[f"gt_paper_pt{pn}_x" for pn in paper_order],
            *[f"gt_paper_pt{pn}_y" for pn in paper_order],
            *[f"gt_paper_pt{pn}_yaw_deg" for pn in paper_order],
            # GT 位姿 (map 坐标系，转换后)
            *[f"gt_map_pt{pn}_x" for pn in paper_order],
            *[f"gt_map_pt{pn}_y" for pn in paper_order],
            *[f"gt_map_pt{pn}_yaw_deg" for pn in paper_order],
            # 逐段 RPE（4段: 2→3, 3→4, 4→1, 1→2）
            "seg_2_to_3_pos_m", "seg_2_to_3_yaw_deg",
            "seg_3_to_4_pos_m", "seg_3_to_4_yaw_deg",
            "seg_4_to_1_pos_m", "seg_4_to_1_yaw_deg",
            "seg_1_to_2_pos_m", "seg_1_to_2_yaw_deg",
            "seg_mean_pos_m", "seg_mean_yaw_deg",
            # 端到端闭合误差
            "loop_slam_dx", "loop_slam_dy", "loop_slam_dyaw_deg",
            "loop_gt_dx", "loop_gt_dy", "loop_gt_dyaw_deg",
            "loop_pos_m", "loop_yaw_deg",
            # 元数据
            "gt_raw_input",
            "notes",
        ]

        logger = AppendingCSVLogger(CSV_RPE_PATH, headers)

        row = [f"{time.time():.3f}"]

        # SLAM 位姿 — 先所有 x，再所有 y，再所有 yaw_deg（匹配表头顺序）
        for i in range(N):
            row.append(f"{self._slam_poses[i][0]:.4f}")
        for i in range(N):
            row.append(f"{self._slam_poses[i][1]:.4f}")
        for i in range(N):
            row.append(f"{math.degrees(self._slam_poses[i][2]):.4f}")

        # GT 论文（原始输入）
        for i in range(N):
            row.append(f"{self._gt_poses_paper[i][0]:.4f}")
        for i in range(N):
            row.append(f"{self._gt_poses_paper[i][1]:.4f}")
        for i in range(N):
            row.append(f"{self._gt_poses_paper[i][2]:.4f}")

        # GT map（转换后）
        for i in range(N):
            row.append(f"{self._gt_poses_map[i][0]:.4f}")
        for i in range(N):
            row.append(f"{self._gt_poses_map[i][1]:.4f}")
        for i in range(N):
            row.append(f"{math.degrees(self._gt_poses_map[i][2]):.4f}")

        # 逐段 RPE
        seg_labels = ["2→3", "3→4", "4→1", "1→2"]
        for i in range(4):
            if i < len(seg_errors_pos):
                row.append(f"{seg_errors_pos[i]:.4f}")
                row.append(f"{math.degrees(seg_errors_yaw[i]):.4f}")
            else:
                row.extend(["", ""])

        # 均值
        mean_pos = (sum(seg_errors_pos) / len(seg_errors_pos)
                    if seg_errors_pos else float("nan"))
        mean_yaw = (sum(seg_errors_yaw) / len(seg_errors_yaw)
                    if seg_errors_yaw else float("nan"))
        row.append(f"{mean_pos:.4f}")
        row.append(f"{math.degrees(mean_yaw):.4f}")

        # 端到端
        row.extend([
            f"{ds_loop_x:.4f}", f"{ds_loop_y:.4f}",
            f"{math.degrees(ds_loop_yaw):.4f}",
            f"{dg_loop_x:.4f}", f"{dg_loop_y:.4f}",
            f"{math.degrees(dg_loop_yaw):.4f}",
            f"{e_loop_pos:.4f}", f"{math.degrees(e_loop_yaw):.4f}",
        ])

        # 元数据
        row.append(gt_raw_str)
        row.append("")

        logger.add_row(row)
        logger.close()

        self.get_logger().info(
            f"RPE 结果已写入 {CSV_RPE_PATH} (run_id={logger.run_id})")

    # ══════════════════════════════════════════════════════════════════════
    # 静态定位稳定性测试
    # ══════════════════════════════════════════════════════════════════════

    def _run_static_test(self):
        self.get_logger().info("========== 静态定位稳定性测试开始 ==========")

        # 确保定位可用
        if self._active_pose_topic is None:
            self.get_logger().error("定位不可用，静态测试中止。")
            return

        # 记录初始位姿
        cp = self._get_current_pose()
        if cp is None:
            self.get_logger().error("无法获取当前位姿。")
            return

        self.get_logger().info(
            f"起始位姿 — map: ({cp[0]:.3f}, {cp[1]:.3f}, {math.degrees(cp[2]):.1f}°)")
        x_p, y_p, yaw_deg_p = map_to_paper(cp[0], cp[1], cp[2])
        self.get_logger().info(
            f"起始位姿 — 论文: ({x_p:.3f}, {y_p:.3f}, {yaw_deg_p:.1f}°)")

        # 稳定等待：丢弃前 2s 内的瞬变帧，避免捕获未完全静止的姿态
        self.get_logger().info("等待机器人完全静止 (3s)...")
        settle_deadline = time.monotonic() + 3.0
        while time.monotonic() < settle_deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # ── 采集静态位姿样本 ────────────────────────────────────────────
        self.get_logger().info(
            f"开始 {STATIC_DURATION_SEC:.0f}s 静止记录，{STATIC_RATE_HZ:.0f}Hz...")
        start_mono = time.monotonic()
        interval = 1.0 / STATIC_RATE_HZ

        csv_logger = CSVLogger(
            POSE_LOG_DIR, "pose_log",
            ["stamp_sec", "stamp_nanosec", "x", "y", "yaw"]
        )
        self.get_logger().info(f"原始数据 -> {csv_logger.filepath}")

        samples = []          # (mono_time, x, y, yaw)
        last_stamp_ns = -1    # 去重：跳过相同时间戳的重复帧

        while time.monotonic() - start_mono < STATIC_DURATION_SEC and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            cp = self._get_current_pose()
            if cp is not None:
                stamp_sec, stamp_ns = self._get_current_stamp()
                stamp_key = stamp_sec * 10**9 + stamp_ns
                if stamp_key == last_stamp_ns:
                    continue  # 跳过重复帧
                last_stamp_ns = stamp_key

                csv_logger.add_row([
                    stamp_sec, stamp_ns,
                    f"{cp[0]:.6f}", f"{cp[1]:.6f}", f"{cp[2]:.6f}",
                ])
                samples.append((time.monotonic(), cp[0], cp[1], cp[2]))
            time.sleep(interval)

        csv_logger.close()

        if len(samples) < 2:
            self.get_logger().error("样本不足，无法计算静态稳定性。")
            return

        self.get_logger().info(
            f"采集完成: {len(samples)} 个样本，"
            f"持续 {samples[-1][0] - samples[0][0]:.1f}s")

        # ── 计算静态稳定性指标 ──────────────────────────────────────────
        self._compute_and_log_static_stability(samples, csv_logger.filepath)

    def _compute_and_log_static_stability(self, samples, raw_csv_path):
        """以第一帧为参考，计算 RMSE 和最大漂移量。"""
        ref_t, ref_x, ref_y, ref_yaw = samples[0]

        pos_deviations = []     # sqrt((x_i - ref_x)² + (y_i - ref_y)²)
        yaw_deviations_rad = [] # |wrap(yaw_i - ref_yaw)|

        for _, x, y, yaw in samples:
            dp = math.sqrt((x - ref_x) ** 2 + (y - ref_y) ** 2)
            dyaw = abs(normalize_angle(yaw - ref_yaw))
            pos_deviations.append(dp)
            yaw_deviations_rad.append(dyaw)

        # RMSE
        rmse_pos = math.sqrt(sum(d ** 2 for d in pos_deviations)
                             / len(pos_deviations))
        rmse_yaw = math.sqrt(sum(d ** 2 for d in yaw_deviations_rad)
                             / len(yaw_deviations_rad))

        # 最大漂移
        max_drift_pos = max(pos_deviations)
        max_drift_yaw = max(yaw_deviations_rad)

        # 统计量
        min_pos = min(pos_deviations)
        max_pos = max(pos_deviations)
        mean_pos = sum(pos_deviations) / len(pos_deviations)

        min_yaw = min(yaw_deviations_rad)
        max_yaw = max(yaw_deviations_rad)
        mean_yaw = sum(yaw_deviations_rad) / len(yaw_deviations_rad)

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("           静态定位稳定性结果")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"  采样数: {len(samples)}   "
            f"时长: {samples[-1][0] - samples[0][0]:.1f}s")
        self.get_logger().info("-" * 60)
        self.get_logger().info(
            f"  {'指标':<16} {'位置(m)':>12} {'航向(°)':>12}")
        self.get_logger().info(
            f"  {'RMSE 抖动':<16} {rmse_pos:>12.6f} {math.degrees(rmse_yaw):>12.4f}")
        self.get_logger().info(
            f"  {'最大漂移':<16} {max_drift_pos:>12.6f} {math.degrees(max_drift_yaw):>12.4f}")
        self.get_logger().info(
            f"  {'最小值':<16} {min_pos:>12.6f} {math.degrees(min_yaw):>12.4f}")
        self.get_logger().info(
            f"  {'最大值':<16} {max_pos:>12.6f} {math.degrees(max_yaw):>12.4f}")
        self.get_logger().info(
            f"  {'平均值':<16} {mean_pos:>12.6f} {math.degrees(mean_yaw):>12.4f}")
        self.get_logger().info("=" * 60 + "\n")

        # ── 写入 CSV ────────────────────────────────────────────────────
        self._write_static_csv(
            rmse_pos, rmse_yaw, max_drift_pos, max_drift_yaw,
            min_pos, max_pos, mean_pos,
            min_yaw, max_yaw, mean_yaw,
            len(samples), raw_csv_path,
        )

    def _write_static_csv(self, rmse_pos, rmse_yaw,
                          max_drift_pos, max_drift_yaw,
                          min_pos, max_pos, mean_pos,
                          min_yaw, max_yaw, mean_yaw,
                          sample_count, raw_csv_path):
        headers = [
            "timestamp",
            "sample_count", "duration_sec",
            "rmse_pos_m", "rmse_yaw_deg",
            "max_drift_pos_m", "max_drift_yaw_deg",
            "min_pos_m", "max_pos_m", "mean_pos_m",
            "min_yaw_deg", "max_yaw_deg", "mean_yaw_deg",
            "raw_csv_path",
            "notes",
        ]

        logger = AppendingCSVLogger(CSV_STATIC_PATH, headers)

        row = [
            f"{time.time():.3f}",
            str(sample_count),
            f"{STATIC_DURATION_SEC:.0f}",
            f"{rmse_pos:.6f}", f"{math.degrees(rmse_yaw):.6f}",
            f"{max_drift_pos:.6f}", f"{math.degrees(max_drift_yaw):.6f}",
            f"{min_pos:.6f}", f"{max_pos:.6f}", f"{mean_pos:.6f}",
            f"{math.degrees(min_yaw):.6f}", f"{math.degrees(max_yaw):.6f}",
            f"{math.degrees(mean_yaw):.6f}",
            raw_csv_path,
            "",
        ]

        logger.add_row(row)
        logger.close()

        self.get_logger().info(
            f"静态稳定性结果已写入 {CSV_STATIC_PATH} (run_id={logger.run_id})")


# ═══════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    parser = argparse.ArgumentParser(
        description="SLAM与导航综合测试 — 闭合路径RPE + 静态定位稳定性")
    parser.add_argument(
        "--mode", choices=["rpe", "static", "all"], default="all",
        help="测试模式: rpe (闭环RPE), static (静态稳定性), all (全部, 默认)")
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = SlamNavTest(mode=args.mode)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断 — 正在关闭...")
    except Exception as exc:
        node.get_logger().error(f"未捕获异常: {exc}")
    finally:
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
