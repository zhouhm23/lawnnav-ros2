#!/usr/bin/env python3
"""
publish_region.py — 将保存的多边形逐点发布到 /clicked_point。

用法（由 launcher 自动调用）:
    python3 launcher/publish_region.py --file ~/.ros/regions/test.yaml

path_coverage 和 coverage_evaluator 会像用户手动点击一样收到这些点。
"""

import argparse
import os
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


def main():
    parser = argparse.ArgumentParser(description="发布保存的覆盖区域")
    parser.add_argument("--file", required=True, help="区域 YAML 文件路径")
    parser.add_argument("--wait", type=int, default=0,
                        help="发布前等待秒数 (默认 0)")
    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"错误: 文件不存在 {args.file}")
        sys.exit(1)

    with open(args.file) as f:
        data = yaml.safe_load(f)

    vertices = data.get("vertices", [])
    frame_id = data.get("frame_id", "map")
    if len(vertices) < 3:
        print(f"错误: 顶点不足 ({len(vertices)})")
        sys.exit(1)

    print(f"发布区域 '{data.get('name', '?')}' ({len(vertices)} 顶点) 到 /clicked_point ...")

    if args.wait > 0:
        print(f"  等待 {args.wait}s 确保订阅者就绪...")
        time.sleep(args.wait)

    rclpy.init()
    node = Node("publish_region", allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True)
    pub = node.create_publisher(PointStamped, "/clicked_point", 10)

    # DDS 发现阶段：等待订阅者连接。
    # 直接创建 publisher 后立即 publish 可能丢失第一条消息，
    # 因为 DDS discovery 需要时间完成端点匹配。
    print("  等待 DDS 订阅者发现 (3s)...")
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() > 0:
            print(f"  ✓ 检测到 {pub.get_subscription_count()} 个订阅者")
            break
    else:
        print("  ⚠ 未检测到订阅者，继续发布（消息可能被缓冲）...")

    # 额外等待 1s 确保订阅者缓冲区就绪
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    for i, (x, y) in enumerate(vertices):
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        pub.publish(msg)
        print(f"  顶点 {i+1}: ({x:.3f}, {y:.3f})")
        # 每点间隔 1.5s，确保 path_coverage 处理完毕再发下一点
        for _ in range(15):
            rclpy.spin_once(node, timeout_sec=0.1)

    # 闭合点
    x0, y0 = vertices[0]
    msg = PointStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.point.x = float(x0)
    msg.point.y = float(y0)
    pub.publish(msg)
    print(f"  闭合 → 多边形应已完成")
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    print("完成 ✓")


if __name__ == "__main__":
    main()
