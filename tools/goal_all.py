#!/usr/bin/env python3
"""
goal_all.py — DEPRECATED: 请使用 test1_slam_nav_test.py --mode all 代替。
Navigate through all 4 goal points sequentially with collision
stuck detection, per-goal timeout, and CSV pose logging at the end.
"""

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
    StuckDetector,
    CmdVelMonitor,
    CSVLogger,
)

GOAL_TIMEOUT_SEC = 120.0


class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__("multi_goal_navigator")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._latest_localization = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self._localization_callback,
            10,
        )

        self._cmd_vel_monitor = CmdVelMonitor(node=self, active_window=1.0)

        self._goals = [
            (1.8, 0.0, -math.pi / 2.0),
            (1.8, -1.0, -math.pi),
            (0.0, -1.0, math.pi / 2.0),
            (0.0, 0.0, 0.0),
        ]
        self._dwell_seconds = 5.0
        self._csv_logger = None

    def _localization_callback(self, msg):
        self._latest_localization = msg

    def _get_current_pose(self):
        if self._latest_localization is None:
            return None
        p = self._latest_localization.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        return (p.position.x, p.position.y, yaw)

    def run(self):
        self.get_logger().info("Waiting for nav2 action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Waiting for localization...")
        while self._latest_localization is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        for idx, (x, y, yaw) in enumerate(self._goals, start=1):
            self.get_logger().info(
                f"Sending goal {idx}/{len(self._goals)}: "
                f"x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
            if not self._send_goal_and_wait(x, y, yaw):
                self.get_logger().error("Goal failed, stopping sequence.")
                return
            time.sleep(self._dwell_seconds)

        self._log_current_pose()
        self._record_pose_samples(rate_hz=5.0, duration_sec=60.0)
        self.get_logger().info("All goals completed.")

    def _send_goal_and_wait(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return False

        result_future = goal_handle.get_result_async()
        stuck_detector = StuckDetector(window_sec=5.0)
        start_time = time.time()
        last_localized_time = start_time

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            cp = self._get_current_pose()
            if cp is not None:
                stuck_detector.update(cp[0], cp[1], cp[2])
                last_localized_time = time.time()
            elif time.time() - last_localized_time > 3.0:
                self.get_logger().error(
                    "Localization lost for 3s — cancelling goal.")
                goal_handle.cancel_goal_async()
                return False

            elapsed = time.time() - start_time
            if (elapsed > 4.0
                    and stuck_detector.is_stuck()
                    and self._cmd_vel_monitor.is_active()):
                self.get_logger().error(
                    "COLLISION STUCK! Cancelling goal.")
                goal_handle.cancel_goal_async()
                return False

            if elapsed > GOAL_TIMEOUT_SEC:
                self.get_logger().error(
                    f"Goal timed out after {GOAL_TIMEOUT_SEC:.0f}s.")
                goal_handle.cancel_goal_async()
                return False

        return result_future.result() is not None

    def _log_current_pose(self):
        cp = self._get_current_pose()
        if cp is not None:
            self.get_logger().info(
                f"Current pose: x={cp[0]:.3f} y={cp[1]:.3f} yaw={cp[2]:.3f}")

    def _record_pose_samples(self, rate_hz, duration_sec):
        self._csv_logger = CSVLogger(
            directory='/home/ubuntu/ros2_ws/src/logs/pose',
            prefix="pose_log",
            headers=["stamp_sec", "stamp_nanosec", "x", "y", "yaw"],
        )
        self.get_logger().info(
            f"Recording {rate_hz:.1f} Hz for {duration_sec:.1f}s "
            f"-> {self._csv_logger.filepath}")
        interval = 1.0 / rate_hz
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)
            cp = self._get_current_pose()
            if cp is not None:
                stamp = self._latest_localization.header.stamp
                self._csv_logger.add_row([
                    stamp.sec, stamp.nanosec,
                    f"{cp[0]:.6f}", f"{cp[1]:.6f}", f"{cp[2]:.6f}",
                ])
            time.sleep(interval)
        self._csv_logger.close()


def main():
    rclpy.init()
    node = MultiGoalNavigator()
    try:
        node.run()
    finally:
        if node._csv_logger is not None:
            node._csv_logger.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
