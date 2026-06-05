#!/usr/bin/env python3
"""
region_capture.py — 从 /clicked_point 捕获多边形并保存。

用法（由 launcher 自动调用）:
    python3 launcher/region_capture.py --name test_region --output ~/.ros/regions/

流程:
    订阅 /clicked_point → 累积顶点 → 多边形闭合 → 保存 .yaml → 退出
"""

import argparse
import math
import os
import sys
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class RegionCapture(Node):
    def __init__(self, name: str, output_dir: str, close_dist: float = 0.08):
        super().__init__("region_capture")
        self._name = name
        self._output_dir = output_dir
        self._close_dist = close_dist
        self._points = []  # list of (x, y)
        self._polygon_closed = False

        self.create_subscription(
            PointStamped, "/clicked_point", self._on_clicked, 10)
        self.get_logger().info(
            f"等待多边形顶点 (点击 ≥3 个点，末点靠近首点自动闭合)...")
        self.get_logger().info(
            f"区域名称: {name}, 闭合距离: {close_dist}m")

    def _on_clicked(self, msg: PointStamped):
        if self._polygon_closed:
            return

        x, y = float(msg.point.x), float(msg.point.y)

        if len(self._points) == 0:
            self._points.append((x, y))
            self.get_logger().info(f"顶点 1: ({x:.3f}, {y:.3f})")
            return

        # 检查是否闭合
        if len(self._points) >= 3:
            x0, y0 = self._points[0]
            if math.hypot(x - x0, y - y0) <= self._close_dist:
                self._polygon_closed = True
                self.get_logger().info(
                    f"多边形闭合 ({len(self._points)} 顶点) → 保存")
                self._save_polygon()
                return

        # 去重
        xl, yl = self._points[-1]
        if math.hypot(x - xl, y - yl) < 0.005:
            return

        self._points.append((x, y))
        self.get_logger().info(f"顶点 {len(self._points)}: ({x:.3f}, {y:.3f})")

    def _save_polygon(self):
        os.makedirs(self._output_dir, exist_ok=True)
        filepath = os.path.join(self._output_dir, f"{self._name}.yaml")
        data = {
            "name": self._name,
            "frame_id": "map",
            "vertices": self._points,
        }
        with open(filepath, "w") as f:
            yaml.dump(data, f, default_flow_style=False)
        self.get_logger().info(f"区域已保存: {filepath}")
        print(f"\033[32m[区域已保存]\033[0m {filepath}")
        # 退出
        raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser(description="捕获多边形区域")
    parser.add_argument("--name", required=True, help="区域名称")
    parser.add_argument("--output", default=os.path.expanduser("~/.ros/regions"),
                        help="输出目录")
    parser.add_argument("--close-dist", type=float, default=0.08,
                        help="多边形闭合距离 (m)")
    args = parser.parse_args()

    rclpy.init()
    node = RegionCapture(args.name, args.output, args.close_dist)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
