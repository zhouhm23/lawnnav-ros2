#!/usr/bin/env python3
"""
run_auto_coverage_test.py — 自动化覆盖作业测试脚本。

用法:
    # 建图 + 覆盖：矩形路径导航建图 → 回起点 → 自动发区域 → 执行覆盖
    python3 tools/run_auto_coverage_test.py --mode mapping

    # 仅覆盖：跳过建图，直接执行覆盖（需已有地图）
    python3 tools/run_auto_coverage_test.py --mode coverage

流程说明:
    1. mapping 模式: 按 test1 矩形路径 (1→2→3→4→1) 导航建图
    2. 回到起点后，自动向 /clicked_point 发布 1.8m×2.4m 区域四角
    3. path_coverage 和 coverage_evaluator 同时接收区域定义并开始覆盖
    4. 脚本监控覆盖执行，完成后输出对照日志
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32
from nav2_msgs.action import NavigateToPose

from test_utils import (
    make_pose_stamped,
    CSVLogger,
)

# ═══════════════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════════════

# 建图矩形路径（map 坐标系，x⁺=车头，y⁺=左侧）
# 对应论文点: 2(0.4,1.8) → 3(1.4,1.8) → 4(1.4,0) → 1(0.4,0)
GOALS_MAP = [
    (1.8,  0.0, -math.pi / 2.0),   # 点2: x_map=y_paper=1.8, y_map=-x_paper+0.4=-0+0.4=0.4 → 不对
    (1.8, -1.0, -math.pi),         # 点3: x_map=1.8, y_map=-1.4+0.4=-1.0
    (0.0, -1.0,  math.pi / 2.0),   # 点4: x_map=0, y_map=-1.4+0.4=-1.0
    (0.0,  0.0,  0.0),             # 点1: x_map=0, y_map=0
]

# 覆盖区域 1.8m×2.4m 的四个角点（map 坐标系）
# 论文区域角: (0,0), (1.8,0), (1.8,2.4), (0,2.4)
# 转换: x_map = y_paper, y_map = -x_paper + 0.4
COVERAGE_REGION_MAP = [
    (0.0,   0.4),    # 论文(0,   0  ): 左下
    (2.4,   0.4),    # 论文(0,   2.4): 左上
    (2.4,  -1.4),    # 论文(1.8, 2.4): 右上
    (0.0,  -1.4),    # 论文(1.8, 0  ): 右下
]

# 单 goal 超时 (s)
GOAL_TIMEOUT_SEC = 120.0
# 覆盖总超时 (s)
COVERAGE_TIMEOUT_SEC = 1800.0  # 30 min
# 覆盖完成判定: cmd_vel 静止超时 (s)
CMD_VEL_IDLE_TIMEOUT = 30.0

# CSV 输出路径
COVERAGE_LOG_DIR = "/home/ubuntu/ros2_ws/src/logs/coverage"

# 定位来源
POSE_TOPIC_CANDIDATES = [
    "/rtabmap/localization_pose",
    "/localization_pose",
]


# ═══════════════════════════════════════════════════════════════════════════
# AutoCoverageTest 节点
# ═══════════════════════════════════════════════════════════════════════════

class AutoCoverageTest(Node):
    def __init__(self, mode: str):
        super().__init__("auto_coverage_test")
        self._mode = mode

        # ── Action client ────────────────────────────────────────────────
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── 订阅 coverage_evaluator 最终覆盖率 ───────────────────────────
        self._final_ratio = None
        self.create_subscription(Float32, "/coverage_ratio", self._ratio_callback, 10)

        # ── 发布 /clicked_point ──────────────────────────────────────────
        self._clicked_pub = self.create_publisher(PointStamped, "/clicked_point", 10)

        # ── 多源定位 ────────────────────────────────────────────────────
        self._pose_cache = {}
        self._active_pose_topic = None
        from geometry_msgs.msg import PoseWithCovarianceStamped
        for topic in POSE_TOPIC_CANDIDATES:
            self.create_subscription(
                PoseWithCovarianceStamped, topic,
                self._make_pose_cb(topic), 10,
            )
            self._pose_cache[topic] = None

        # ── CSV logger ──────────────────────────────────────────────────
        self._csv = CSVLogger(COVERAGE_LOG_DIR, "auto_coverage_log",
                              ["mode", "mapping_time_s", "coverage_time_s",
                               "coverage_evaluator_ratio", "notes"])

        self.get_logger().info(f"AutoCoverageTest 初始化 (mode={mode})")

    # ── 回调 ─────────────────────────────────────────────────────────────

    def _make_pose_cb(self, topic: str):
        def cb(msg):
            self._pose_cache[topic] = msg
            if self._active_pose_topic is None:
                self._active_pose_topic = topic
        return cb

    def _ratio_callback(self, msg: Float32):
        self._final_ratio = float(msg.data)

    def _get_current_pose(self):
        for topic in POSE_TOPIC_CANDIDATES:
            msg = self._pose_cache.get(topic)
            if msg is not None:
                return (msg.pose.pose.position.x,
                        msg.pose.pose.position.y)
        return None

    # ── 定位等待 ─────────────────────────────────────────────────────────

    def _wait_localization(self, timeout: float = 15.0) -> bool:
        self.get_logger().info(
            f"等待定位数据 (候选: {', '.join(POSE_TOPIC_CANDIDATES)})...")
        deadline = time.time() + timeout
        while self._active_pose_topic is None and rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._active_pose_topic is None:
            self.get_logger().error(f"定位超时 ({timeout}s)")
            return False
        self.get_logger().info(f"定位就绪: {self._active_pose_topic}")
        cp = self._get_current_pose()
        if cp:
            self.get_logger().info(f"初始位姿: ({cp[0]:.3f}, {cp[1]:.3f})")
        return True

    # ── 导航到目标 ───────────────────────────────────────────────────────

    def _send_goal_and_wait(self, x: float, y: float, yaw: float) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝目标")
            return False
        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > GOAL_TIMEOUT_SEC:
                self.get_logger().error("目标超时 — 取消")
                goal_handle.cancel_goal_async()
                return False
        result = result_future.result()
        ok = result is not None
        if ok:
            self.get_logger().info("  ✓ 到达")
        return ok

    # ── 建图 ─────────────────────────────────────────────────────────────

    def _run_mapping(self) -> bool:
        """按矩形路径导航建图，返回是否成功。"""
        self.get_logger().info("========== 建图阶段 ==========")
        self.get_logger().info("等待 Nav2 action server...")
        self._action_client.wait_for_server()
        t0 = time.time()
        for idx, (gx, gy, gyaw) in enumerate(GOALS_MAP, start=1):
            self.get_logger().info(
                f"建图目标 {idx}/{len(GOALS_MAP)}: ({gx:.2f}, {gy:.2f})")
            if not self._send_goal_and_wait(gx, gy, gyaw):
                self.get_logger().error(f"建图目标 {idx} 失败，中止")
                return False
        elapsed = time.time() - t0
        self.get_logger().info(f"建图完成 ✓ 耗时 {elapsed:.1f}s")
        self._mapping_time = elapsed
        return True

    # ── 发布覆盖区域 ─────────────────────────────────────────────────────

    def _publish_coverage_region(self):
        """向 /clicked_point 按序发布四个角点 + 闭合点。"""
        self.get_logger().info("发布覆盖区域角点到 /clicked_point...")
        # 按序发布四个角点
        for i, (cx, cy) in enumerate(COVERAGE_REGION_MAP):
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = float(cx)
            msg.point.y = float(cy)
            msg.point.z = 0.0
            self._clicked_pub.publish(msg)
            self.get_logger().info(f"  角点 {i+1}: ({cx:.3f}, {cy:.3f})")
            # 留间隔让订阅者有时间处理（path_coverage 需要逐步接收）
            time.sleep(0.5)
            # spin 以让消息真正发出去
            rclpy.spin_once(self, timeout_sec=0.05)

        # 发布闭合点（靠近第一个角点，触发多边形闭合）
        cx0, cy0 = COVERAGE_REGION_MAP[0]
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(cx0)
        msg.point.y = float(cy0)
        msg.point.z = 0.0
        self._clicked_pub.publish(msg)
        self.get_logger().info(f"  闭合点: ({cx0:.3f}, {cy0:.3f}) — 多边形应闭合")

    # ── 等待覆盖完成 ─────────────────────────────────────────────────────

    def _wait_coverage_complete(self) -> float:
        """等待覆盖完成（cmd_vel 持续静止），返回覆盖耗时 (s)。

        策略: 持续 spin，监控 /coverage_ratio。如果 cmd_vel 长期无效
        则无法直接订阅；简化为超时等待。用户也可 Ctrl+C 提前结束。
        """
        self.get_logger().info(
            f"等待覆盖完成（总超时 {COVERAGE_TIMEOUT_SEC}s，"
            f"也可 Ctrl+C 提前结束）...")
        t_start = time.time()
        log_interval = 30.0  # 每 30s 打印一次覆盖率状态
        last_log = t_start

        while rclpy.ok() and (time.time() - t_start) < COVERAGE_TIMEOUT_SEC:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.time()
            if now - last_log >= log_interval:
                ratio = self._final_ratio or 0.0
                elapsed = now - t_start
                self.get_logger().info(
                    f"  [{elapsed:.0f}s] 当前覆盖率: {ratio*100:.1f}% "
                    f"(coverage_evaluator)")
                last_log = now

        elapsed = time.time() - t_start
        ratio = self._final_ratio or 0.0
        self.get_logger().info(
            f"覆盖阶段结束，耗时 {elapsed:.1f}s，"
            f"coverage_evaluator 报告覆盖率: {ratio*100:.2f}%")
        return elapsed

    # ── 主流程 ────────────────────────────────────────────────────────────

    def run(self):
        # 1. 等待定位
        if not self._wait_localization():
            return

        mapping_time = 0.0
        notes = ""

        # 2. 建图（仅 mapping 模式）
        if self._mode == "mapping":
            if not self._run_mapping():
                notes = "mapping failed"
                self._csv.add_row([self._mode, 0.0, 0.0, 0.0, notes])
                self._csv.close()
                return
            mapping_time = getattr(self, '_mapping_time', 0.0)
            notes = "mapping ok"
        else:
            notes = "coverage only (reuse map)"

        # 3. 小等一下，让系统稳定
        self.get_logger().info("等待 2s 让系统稳定...")
        time.sleep(2.0)

        # 4. 发布覆盖区域（path_coverage 和 coverage_evaluator 同时接收）
        self._publish_coverage_region()

        # 5. 再等一下让两个节点完成多边形闭合
        time.sleep(1.0)

        # 6. 等待覆盖执行完成
        coverage_time = self._wait_coverage_complete()

        # 7. 最终日志
        final_ratio = self._final_ratio or 0.0
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  覆盖测试完成\n"
            f"  模式: {self._mode}\n"
            f"  建图耗时: {mapping_time:.1f}s\n"
            f"  覆盖耗时: {coverage_time:.1f}s\n"
            f"  coverage_evaluator 覆盖率: {final_ratio*100:.2f}%\n"
            f"{'='*60}"
        )

        self._csv.add_row([
            self._mode, f"{mapping_time:.1f}", f"{coverage_time:.1f}",
            f"{final_ratio:.4f}", notes
        ])
        self._csv.close()


# ═══════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="自动化覆盖作业测试")
    parser.add_argument("--mode", choices=["mapping", "coverage"],
                        default="mapping",
                        help="mapping: 先建图再覆盖 | coverage: 仅覆盖")
    args = parser.parse_args()

    rclpy.init()
    node = AutoCoverageTest(mode=args.mode)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
        final = node._final_ratio or 0.0
        node.get_logger().info(f"coverage_evaluator 最终覆盖率: {final*100:.2f}%")
        if hasattr(node, '_csv') and node._csv:
            try:
                mapping_t = getattr(node, '_mapping_time', 0.0)
                node._csv.add_row([args.mode, f"{mapping_t:.1f}", "interrupted",
                                   f"{final:.4f}", "interrupted"])
            except Exception:
                pass
    finally:
        try:
            if hasattr(node, '_csv'):
                node._csv.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
