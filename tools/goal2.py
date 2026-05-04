#!/usr/bin/env python3
"""
goal2.py — DEPRECATED: 请使用 test1_slam_nav_test.py --mode rpe 代替。
Navigate to a single goal point with collision stuck detection and timeout.
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
)

GOAL_TIMEOUT_SEC = 120.0


class SingleGoalNavigator(Node):
    def __init__(self):
        super().__init__("goal_navigator_2")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._latest_localization = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self._localization_callback,
            10,
        )

        self._cmd_vel_monitor = CmdVelMonitor(node=self, active_window=1.0)

        self._goal_xy = (1.8, -1.0)
        self._goal_yaw = -math.pi
        self._dwell_seconds = 10.0

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

        x, y = self._goal_xy
        yaw = self._goal_yaw

        self.get_logger().info(
            f"Sending goal: x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by Nav2")
            return

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
                self.get_logger().error("Localization lost for 3s — cancelling goal.")
                goal_handle.cancel_goal_async()
                return

            elapsed = time.time() - start_time
            if (elapsed > 4.0
                    and stuck_detector.is_stuck()
                    and self._cmd_vel_monitor.is_active()):
                self.get_logger().error("COLLISION STUCK! Cancelling goal.")
                goal_handle.cancel_goal_async()
                return

            if elapsed > GOAL_TIMEOUT_SEC:
                self.get_logger().error(
                    f"Goal timed out after {GOAL_TIMEOUT_SEC:.0f}s.")
                goal_handle.cancel_goal_async()
                return

        result = result_future.result()
        if result is None:
            self.get_logger().error("Goal failed.")
            return

        self.get_logger().info(
            f"Goal reached. Dwelling for {self._dwell_seconds:.1f}s...")
        time.sleep(self._dwell_seconds)

        cp = self._get_current_pose()
        if cp is not None:
            self.get_logger().info(
                f"Final pose: x={cp[0]:.3f} y={cp[1]:.3f} yaw={cp[2]:.3f}")
        self.get_logger().info("Done.")


def main():
    rclpy.init()
    node = SingleGoalNavigator()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
