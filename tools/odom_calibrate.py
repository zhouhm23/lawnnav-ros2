#!/usr/bin/env python3
"""
odom_calibrate.py — 里程计系统误差校准。

用法:
    python3 tools/odom_calibrate.py

前提:
    1. 手动启动建图+导航 (ros2 launch navigation rtabmap_camera_nav.launch.py)
    2. 车放在起点 (0, 0)

流程:
    起点 → P1(2.0, 0)   纯 x 前进 2m  → 输入 GT
         → P2(2.0, -1.0) 纯 y 侧移 1m  → 输入 GT
         → 起点(0, 0)    对角线闭合     → 输入 GT

    每到达一个点: 打印里程计读数 → 等待用户输入实际测量值
    最后输出 x/y/yaw 系统修正系数。
"""

import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

from test_utils import (
    yaw_from_quaternion,
    make_pose_stamped,
    normalize_angle,
)

# ── 校准点 (map 坐标系) ─────────────────────────────────────────────
GOALS = [
    (2.0,  0.0,  0.0),   # P1: 纯 x 前进 2m
    (2.0, -1.0,  0.0),   # P2: 纯 y 侧移 1m
    (0.0,  0.0,  0.0),   # P3: 闭合 (对角线回起点)
]
GOAL_LABELS = ["P1(2.0, 0)", "P2(2.0, -1.0)", "起点(0, 0)"]
GOAL_TIMEOUT_SEC = 180.0

# ── odom 订阅 ───────────────────────────────────────────────────────
ODOM_TOPIC = "/odom"


def _info(m): print(f"\033[36m[INFO]\033[0m {m}")
def _ok(m): print(f"\033[32m[OK]\033[0m {m}")
def _warn(m): print(f"\033[33m[WARN]\033[0m {m}")


class OdomCalibrator(Node):
    def __init__(self):
        super().__init__("odom_calibrator")
        self._odom_msg = None
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_callback, 10)

        self._odom_readings = []  # [(x, y, yaw_deg)]
        self._gt_inputs = []      # [(x, y, yaw_deg)]

    def _odom_callback(self, msg):
        self._odom_msg = msg

    def _get_odom(self):
        if self._odom_msg is None:
            return None
        p = self._odom_msg.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        return (p.position.x, p.position.y, math.degrees(yaw))

    def _send_goal_and_wait(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            _warn("目标被 Nav2 拒绝")
            return False

        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > GOAL_TIMEOUT_SEC:
                _warn("超时")
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        return result is not None

    def run(self):
        _info("等待 Nav2 action server...")
        if not self._action_client.wait_for_server(timeout_sec=60):
            _warn("Nav2 未就绪，请先启动 navigation")
            return

        _info(f"订阅 /odom topic，等待数据...")
        while self._odom_msg is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        _ok("/odom 数据就绪")

        # ── 逐点导航 + 记录 ─────────────────────────────────────────
        for i, (gx, gy, gyaw) in enumerate(GOALS):
            label = GOAL_LABELS[i]
            _info(f"导航到 {label} ...")
            ok = self._send_goal_and_wait(gx, gy, gyaw)
            if not ok:
                _warn(f"导航到 {label} 失败")
                return

            time.sleep(1.0)  # 等里程计稳定
            rclpy.spin_once(self, timeout_sec=0.1)
            odom = self._get_odom()
            if odom is None:
                _warn("里程计数据丢失")
                return

            ox, oy, oyaw = odom
            self._odom_readings.append(odom)
            print(f"\n{'='*50}")
            _ok(f"到达 {label}")
            print(f"  里程计报告:  x={ox:.3f}  y={oy:.3f}  yaw={oyaw:.1f}°")
            print(f"  请输入实际测量值:")
            gt_str = input("  实际 (x y yaw°): ").strip()
            try:
                parts = gt_str.split()
                gx_in = float(parts[0])
                gy_in = float(parts[1])
                gyaw_in = float(parts[2]) if len(parts) > 2 else 0.0
            except (ValueError, IndexError):
                _warn(f"格式错误: '{gt_str}'，预期 'x y yaw°'")
                return
            self._gt_inputs.append((gx_in, gy_in, gyaw_in))
            print()

        # ── 计算修正系数 ─────────────────────────────────────────────
        self._compute_calibration()

    def _compute_calibration(self):
        N = len(self._gt_inputs)
        if N < 2:
            _warn("数据不足")
            return

        # ── 逐段分析 ─────────────────────────────────────────────────
        x_scales = []
        y_scales = []
        yaw_offsets = []
        segment_distances = []  # 各段 GT 距离，用于算 yaw 漂移率

        for i in range(N):
            ox, oy, oyaw = self._odom_readings[i]
            gx, gy, gyaw = self._gt_inputs[i]

            # 相对于上一段起点的位移（用于算每段误差）
            if i == 0:
                prev_gx, prev_gy = 0.0, 0.0
                prev_ox, prev_oy = 0.0, 0.0
            else:
                prev_gx, prev_gy, _ = self._gt_inputs[i - 1]
                prev_ox, prev_oy, _ = self._odom_readings[i - 1]

            dgx = gx - prev_gx
            dgy = gy - prev_gy
            dox = ox - prev_ox
            doy = oy - prev_oy
            seg_dist = math.sqrt(dgx ** 2 + dgy ** 2)

            if abs(dox) > 0.03:
                x_scales.append(dgx / dox)
            if abs(doy) > 0.03:
                y_scales.append(dgy / doy)
            yaw_offsets.append(gyaw - oyaw)
            if seg_dist > 0.1:
                segment_distances.append((seg_dist, gyaw - oyaw))

        scale_x = sum(x_scales) / len(x_scales) if x_scales else 1.0
        scale_y = sum(y_scales) / len(y_scales) if y_scales else 1.0
        yaw_offset = sum(yaw_offsets) / len(yaw_offsets) if yaw_offsets else 0.0

        # 每米 yaw 漂移量（°/m）
        yaw_drift_per_m = 0.0
        if segment_distances:
            total_dist = sum(d for d, _ in segment_distances)
            total_yaw_drift = sum(abs(y) for _, y in segment_distances)
            if total_dist > 0.1:
                yaw_drift_per_m = total_yaw_drift / total_dist

        # ── 逐项输出 ─────────────────────────────────────────────────
        print(f"\n{'='*60}")
        print(f"  里程计校准结果（逐段位移差分析）")
        print(f"{'='*60}")
        print(f"{'段':>14} {'GT Δx':>8} {'GT Δy':>8} {'Odom Δx':>8} {'Odom Δy':>8} "
              f"{'x比':>8} {'y比':>8} {'yaw差°':>8}")
        print(f"{'-'*60}")
        for i in range(N):
            if i == 0:
                prev_gx, prev_gy, _ = 0.0, 0.0, 0.0
                prev_ox, prev_oy, _ = 0.0, 0.0, 0.0
            else:
                prev_gx, prev_gy, _ = self._gt_inputs[i - 1]
                prev_ox, prev_oy, _ = self._odom_readings[i - 1]

            gx, gy, _ = self._gt_inputs[i]
            ox, oy, oyaw = self._odom_readings[i]
            dgx, dgy = gx - prev_gx, gy - prev_gy
            dox, doy = ox - prev_ox, oy - prev_oy
            yr = dgy / doy if abs(doy) > 0.03 else float('nan')
            xr = dgx / dox if abs(dox) > 0.03 else float('nan')
            yd = self._gt_inputs[i][2] - oyaw
            label = GOAL_LABELS[i].split('(')[0]
            print(f"{label:>14} {dgx:>8.3f} {dgy:>8.3f} {dox:>8.3f} {doy:>8.3f} "
                  f"{xr:>8.4f} {yr:>8.4f} {yd:>8.1f}")

        print(f"\n{'='*60}")
        print(f"  汇总修正系数（建议写入 odom_publisher_node.py）")
        print(f"{'='*60}")
        print(f"  x 方向缩放 (odom_x *= ...):  {scale_x:.4f}   (odom 偏高 {abs(100*(1-scale_x)):.1f}%)")
        print(f"  y 方向缩放 (odom_y *= ...):  {scale_y:.4f}   (odom 偏高 {abs(100*(1-scale_y)):.1f}%)")
        print(f"  yaw 累计偏移 (°/m):         {yaw_drift_per_m:.2f}°/m  (IMU≤1°时可忽略)")
        print(f"{'='*60}")

        # ── 代码修改建议 ─────────────────────────────────────────────
        print(f"""
  代码位置: driver/controller/controller/odom_publisher_node.py
  函数 cal_odom_fun() 第 ~270 行，将:

    self.odom.pose.pose.position.x = self.linear_factor * self.x
    self.odom.pose.pose.position.y = self.linear_factor * self.y

  改为:
""")
        # 如果 x/y 接近，建议用平均值；如果差异大，建议分开
        if abs(scale_x - scale_y) < 0.02:
            avg = (scale_x + scale_y) / 2.0
            print(f"    odom_factor_x = self.linear_factor * {avg:.4f}")
            print(f"    odom_factor_y = self.linear_factor * {avg:.4f}")
            print(f"    self.odom.pose.pose.position.x = odom_factor_x * self.x")
            print(f"    self.odom.pose.pose.position.y = odom_factor_y * self.y")
        else:
            print(f"    odom_factor_x = self.linear_factor * {scale_x:.4f}")
            print(f"    odom_factor_y = self.linear_factor * {scale_y:.4f}")
            print(f"    self.odom.pose.pose.position.x = odom_factor_x * self.x")
            print(f"    self.odom.pose.pose.position.y = odom_factor_y * self.y")

        if yaw_drift_per_m > 0.5:
            print(f"""
  如果 yaw 漂移较大 (>{0.5}°/m)，还应修正角速度累积:
    self.pose_yaw += self.angular_factor * delta_yaw  # angular_factor={1.0 + yaw_drift_per_m/100:.4f}
""")
        else:
            print(f"\n  yaw 漂移 ≤ 0.5°/m，角速度修正可忽略（IMU yaw 已足够准）")


def main():
    rclpy.init()
    node = OdomCalibrator()
    try:
        node.run()
    except KeyboardInterrupt:
        _info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
