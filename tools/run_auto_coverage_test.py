#!/usr/bin/env python3
"""
run_auto_coverage_test.py — 自动化覆盖作业测试脚本。

用法:
    python3 tools/run_auto_coverage_test.py --mode mapping   # 手动建图 → 保存 → 覆盖
    python3 tools/run_auto_coverage_test.py --mode coverage  # 复用地图直接覆盖
    python3 tools/run_auto_coverage_test.py --list-maps      # 列出地图备份

mapping 模式:
    1. 用户手动控制小车建图（脚本等待终端回车）
    2. 自动保存地图（拷贝 rtabmap.db 到 ~/.ros/maps/）
    3. 提示用户重启终端1为 localization:=true
    4. 用户回车后 → 自动发 1.8×2.4m 覆盖区域 → 监控覆盖

coverage 模式:
    1. 等待定位 → 自动发覆盖区域 → 监控覆盖
"""

import argparse
import os
import shutil
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

from test_utils import CSVLogger

# ═══════════════════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════════════════

# 覆盖区域 1.8m×2.4m 四个角点（map 坐标系）
# 论文→map: x_map=y_paper, y_map=-x_paper+0.4
COVERAGE_REGION_MAP = [
    (0.0,   0.4),    # 论文(0, 0)
    (2.4,   0.4),    # 论文(0, 2.4)
    (2.4,  -1.4),    # 论文(1.8, 2.4)
    (0.0,  -1.4),    # 论文(1.8, 0)
]

COVERAGE_TIMEOUT_SEC = 1800.0   # 30 min
CSV_DIR = str(Path.home() / "ros2_ws" / "src" / "logs" / "coverage")
MAP_BACKUP_DIR = str(Path.home() / ".ros" / "maps")
RTABMAP_DB = str(Path.home() / ".ros" / "rtabmap.db")

POSE_TOPIC_CANDIDATES = [
    "/rtabmap/localization_pose",
    "/localization_pose",
]


# ═══════════════════════════════════════════════════════════════════════════
# 地图管理（纯函数，无 ROS 依赖）
# ═══════════════════════════════════════════════════════════════════════════

def save_map_backup(name: str = "") -> str:
    os.makedirs(MAP_BACKUP_DIR, exist_ok=True)
    if not name:
        name = time.strftime("auto_%Y%m%d_%H%M%S")
    dst = os.path.join(MAP_BACKUP_DIR, f"{name}.db")
    if os.path.exists(RTABMAP_DB):
        shutil.copy2(RTABMAP_DB, dst)
        print(f"\033[32m[地图已保存]\033[0m {dst} "
              f"({os.path.getsize(dst)/1024/1024:.1f} MB)")
    else:
        print(f"\033[33m[警告]\033[0m {RTABMAP_DB} 不存在")
    return dst


def list_map_backups() -> list:
    if not os.path.isdir(MAP_BACKUP_DIR):
        return []
    return sorted([f for f in os.listdir(MAP_BACKUP_DIR) if f.endswith('.db')],
                  reverse=True)


# ═══════════════════════════════════════════════════════════════════════════
# AutoCoverageTest 节点
# ═══════════════════════════════════════════════════════════════════════════

class AutoCoverageTest(Node):
    def __init__(self, mode: str):
        super().__init__("auto_coverage_test")
        self._mode = mode
        self._final_ratio: float = 0.0

        self.create_subscription(Float32, "/coverage_ratio",
                                 self._ratio_callback, 10)
        self._clicked_pub = self.create_publisher(
            PointStamped, "/clicked_point", 10)

        self._pose_cache = {}
        self._active_pose_topic = None
        from geometry_msgs.msg import PoseWithCovarianceStamped
        for topic in POSE_TOPIC_CANDIDATES:
            self.create_subscription(PoseWithCovarianceStamped, topic,
                                     self._make_pose_cb(topic), 10)
            self._pose_cache[topic] = None

        self._csv = CSVLogger(CSV_DIR, "auto_coverage_log",
                              ["mode", "coverage_time_s",
                               "coverage_evaluator_ratio", "notes"])
        self.get_logger().info(f"AutoCoverageTest init (mode={mode})")

    def _make_pose_cb(self, topic: str):
        def cb(msg):
            self._pose_cache[topic] = msg
            if self._active_pose_topic is None:
                self._active_pose_topic = topic
        return cb

    def _ratio_callback(self, msg: Float32):
        self._final_ratio = float(msg.data)

    def _wait_localization(self, timeout: float = 30.0) -> bool:
        self.get_logger().info("等待定位数据...")
        dl = time.time() + timeout
        while self._active_pose_topic is None and rclpy.ok() and time.time() < dl:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._active_pose_topic is None:
            self.get_logger().error(f"定位超时 ({timeout}s)")
            return False
        self.get_logger().info(f"定位就绪: {self._active_pose_topic}")
        return True

    def _publish_coverage_region(self):
        self.get_logger().info("发布覆盖区域角点到 /clicked_point ...")
        for i, (cx, cy) in enumerate(COVERAGE_REGION_MAP):
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = float(cx)
            msg.point.y = float(cy)
            self._clicked_pub.publish(msg)
            self.get_logger().info(f"  角点 {i+1}: ({cx:.3f}, {cy:.3f})")
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.05)
        cx0, cy0 = COVERAGE_REGION_MAP[0]
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(cx0)
        msg.point.y = float(cy0)
        self._clicked_pub.publish(msg)
        self.get_logger().info("  闭合 → 多边形应已完成")

    def _wait_coverage_complete(self) -> float:
        self.get_logger().info(
            f"等待覆盖完成（总超时 {COVERAGE_TIMEOUT_SEC}s，可 Ctrl+C）...")
        t0 = time.time()
        last_log = t0
        while rclpy.ok() and (time.time() - t0) < COVERAGE_TIMEOUT_SEC:
            rclpy.spin_once(self, timeout_sec=0.5)
            now = time.time()
            if now - last_log >= 30.0:
                self.get_logger().info(
                    f"  [{now-t0:.0f}s] coverage_evaluator: "
                    f"{self._final_ratio*100:.1f}%")
                last_log = now
        elapsed = time.time() - t0
        self.get_logger().info(
            f"覆盖结束，耗时 {elapsed:.1f}s，"
            f"coverage_evaluator: {self._final_ratio*100:.2f}%")
        return elapsed

    # ── mapping 模式 ─────────────────────────────────────────────────

    def _run_mapping_mode(self):
        print()
        print("\033[1;36m╔══════════════════════════════════════════════╗\033[0m")
        print("\033[1;36m║  建图阶段 — 请手动控制小车遍历工作区域       ║\033[0m")
        print("\033[1;36m║  建议: 沿 1→2→3→4→1 行驶后停在论文起点      ║\033[0m")
        print("\033[1;36m╚══════════════════════════════════════════════╝\033[0m")
        print()
        input("\033[1m建图完成后按 Enter 保存地图...\033[0m")

        print()
        ts = time.strftime("%Y%m%d_%H%M%S")
        save_map_backup(f"mapping_{ts}")
        print()

        print("\033[1;33m╔══════════════════════════════════════════════╗\033[0m")
        print("\033[1;33m║  请重启终端1为纯定位模式后按 Enter:          ║\033[0m")
        print("\033[1;33m║    ~/.stop_ros.sh                            ║\033[0m")
        print("\033[1;33m║    ros2 launch navigation \\                  ║\033[0m")
        print("\033[1;33m║      rtabmap_navigation.launch.py \\           ║\033[0m")
        print("\033[1;33m║      localization:=true                      ║\033[0m")
        print("\033[1;33m║  或: launcher 输入 > coverage                ║\033[0m")
        print("\033[1;33m╚══════════════════════════════════════════════╝\033[0m")
        input("\033[1m就绪后按 Enter 开始覆盖测试...\033[0m")
        print()
        self._run_coverage_phase()

    # ── coverage 模式 ────────────────────────────────────────────────

    def _run_coverage_mode(self):
        if not os.path.exists(RTABMAP_DB):
            self.get_logger().warn(f"{RTABMAP_DB} 不存在")
            backups = list_map_backups()
            if backups:
                print(f"\n可用地图备份 ({MAP_BACKUP_DIR}):")
                for b in backups[:5]:
                    sz = os.path.getsize(os.path.join(MAP_BACKUP_DIR, b))
                    print(f"  {b}  ({sz/1024/1024:.1f} MB)")
            print()
        self._run_coverage_phase()

    def _run_coverage_phase(self):
        if not self._wait_localization():
            return
        self.get_logger().info("等待 3s 确保节点就绪...")
        time.sleep(3.0)
        self._publish_coverage_region()
        time.sleep(1.0)
        ct = self._wait_coverage_complete()
        fr = self._final_ratio or 0.0

        self.get_logger().info(
            f"\n{'='*55}\n"
            f"  覆盖测试完成  模式: {self._mode}  耗时: {ct:.1f}s\n"
            f"  coverage_evaluator 覆盖率: {fr*100:.2f}%\n"
            f"{'='*55}")
        self._csv.add_row([self._mode, f"{ct:.1f}", f"{fr:.4f}", ""])
        self._csv.close()

    def run(self):
        if self._mode == "mapping":
            self._run_mapping_mode()
        else:
            self._run_coverage_mode()


# ═══════════════════════════════════════════════════════════════════════════
# main
# ═══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="自动化覆盖作业测试")
    parser.add_argument("--mode", choices=["mapping", "coverage"],
                        default="mapping")
    parser.add_argument("--list-maps", action="store_true",
                        help="列出所有地图备份")
    args = parser.parse_args()

    if args.list_maps:
        backups = list_map_backups()
        if backups:
            print(f"\n地图备份 ({MAP_BACKUP_DIR}):")
            for b in backups:
                sz = os.path.getsize(os.path.join(MAP_BACKUP_DIR, b))
                print(f"  {b}  ({sz/1024/1024:.1f} MB)")
        else:
            print("无备份")
        sys.exit(0)

    rclpy.init()
    node = AutoCoverageTest(mode=args.mode)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
        fr = node._final_ratio or 0.0
        node.get_logger().info(f"coverage_evaluator: {fr*100:.2f}%")
        if hasattr(node, '_csv') and node._csv:
            try:
                node._csv.add_row(
                    [args.mode, "interrupted", f"{fr:.4f}", "interrupted"])
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
