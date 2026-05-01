#!/usr/bin/env python3
"""
test2_measure_trajectory_error.py
Navigate through 4 goal points continuously while logging
theoretical-vs-actual trajectory error at 1 Hz.
"""

import math
import sys
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose

from test_utils import (
    yaw_from_quaternion,
    make_pose_stamped,
    normalize_angle,
    rotate_360,
    CSVLogger,
)

GOAL_TIMEOUT_SEC = 120.0  # per-goal navigation timeout


class MeasureTrajectoryError(Node):
    def __init__(self):
        super().__init__("measure_trajectory_error")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._latest_localization = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/localization_pose",
            self._localization_callback,
            10,
        )

        self._goals = [
            (1.8, 0.0, -math.pi / 2.0),
            (1.8, -1.0, -math.pi),
            (0.0, -1.0, math.pi / 2.0),
            (0.0, 0.0, 0.0),
        ]

        self.max_v = 0.26
        self.max_w = 1.0
        self._load_velocity_params()

        self._csv_logger = None
        self._current_segment_start_pose = None
        self._current_segment_goal_pose = None
        self._current_segment_start_time = None

    # ---- parameter loading -------------------------------------------------

    def _load_velocity_params(self):
        yaml_path = "/home/ubuntu/ros2_ws/src/navigation/config/nav2_params.yaml"
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
                vs = data.get('velocity_smoother', {}).get('ros__parameters', {})
                if 'max_velocity' in vs:
                    max_vels = vs['max_velocity']
                    self.max_v = float(max_vels[0])
                    self.max_w = float(max_vels[2])
        except Exception as e:
            self.get_logger().warn(
                f"Failed to parse yaml, using defaults v={self.max_v} w={self.max_w}: {e}")

    # ---- pose helpers ------------------------------------------------------

    def _localization_callback(self, msg):
        self._latest_localization = msg

    def _get_current_pose(self):
        if self._latest_localization is None:
            return None
        p = self._latest_localization.pose.pose
        yaw = yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                  p.orientation.z, p.orientation.w)
        return (p.position.x, p.position.y, yaw)

    def _get_current_yaw(self):
        if self._latest_localization is None:
            return None
        p = self._latest_localization.pose.pose
        return yaw_from_quaternion(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w)

    # ---- main --------------------------------------------------------------

    def run(self):
        self.get_logger().info(
            f"Loaded constraints: max_v={self.max_v:.2f}, max_w={self.max_w:.2f}")
        self.get_logger().info("Waiting for nav2 action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Waiting for localization...")
        while self._latest_localization is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # --- closed-loop 360 rotation ---------------------------------------
        # Brief stop before rotation to kill any residual motion
        stop_twist = Twist()
        self._cmd_vel_pub.publish(stop_twist)
        time.sleep(0.5)

        ok = rotate_360(self, self._cmd_vel_pub, self._get_current_yaw,
                        angular_speed=0.5, timeout=30.0)
        if not ok:
            self.get_logger().warn("Rotation incomplete — map may have blind spots")

        # Let Nav2 take over /cmd_vel before we start sending goals.
        # Without this, the last zero-velocity stop from rotate_360 can
        # race with Nav2's first motion command.
        self.get_logger().info("Waiting 2s for Nav2 to take over cmd_vel...")
        deadline = time.time() + 2.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        # --- CSV logger -----------------------------------------------------
        self._csv_logger = CSVLogger(
            directory='/home/ubuntu/ros2_ws/src/tools',
            prefix="trajectory_error",
            headers=[
                "timestamp", "segment_time",
                "actual_x", "actual_y", "actual_yaw",
                "theo_x", "theo_y", "theo_yaw",
                "err_x", "err_y", "err_yaw",
            ],
        )
        self.get_logger().info(f"CSV -> {self._csv_logger.filepath}")

        # --- navigate through goals -----------------------------------------
        for idx, (x, y, yaw) in enumerate(self._goals, start=1):
            self.get_logger().info(
                f"Sending goal {idx}/{len(self._goals)}: "
                f"x={x:.2f} y={y:.2f} yaw={yaw:.2f}")

            cp = self._get_current_pose()
            if cp is None:
                self.get_logger().error("Lost localization, cannot continue!")
                break

            self._current_segment_start_pose = cp
            self._current_segment_goal_pose = (x, y, yaw)
            self._current_segment_start_time = time.time()

            if not self._send_goal_and_wait(x, y, yaw):
                self.get_logger().error("Goal failed, stopping sequence.")
                break

        self._csv_logger.close()
        self.get_logger().info(
            f"All goals completed. Data -> {self._csv_logger.filepath}")

    # ---- navigation --------------------------------------------------------

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
        last_sample_time = time.time()
        start_time = time.time()
        last_localized_time = start_time
        LOCALIZATION_TIMEOUT = 3.0

        # Initial sample
        self._sample_data()

        while not result_future.done() and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()

            # Track localization health
            cp = self._get_current_pose()
            if cp is not None:
                last_localized_time = now
            elif now - last_localized_time > LOCALIZATION_TIMEOUT:
                self.get_logger().error(
                    f"Localization lost for {LOCALIZATION_TIMEOUT:.0f}s — "
                    "cancelling goal.")
                goal_handle.cancel_goal_async()
                return False

            # 1 Hz sampling
            if now - last_sample_time >= 1.0:
                self._sample_data()
                last_sample_time = now

            # Per-goal timeout
            if now - start_time > GOAL_TIMEOUT_SEC:
                self.get_logger().error(
                    f"Goal timed out after {GOAL_TIMEOUT_SEC:.0f}s. Cancelling.")
                goal_handle.cancel_goal_async()
                return False

        result = result_future.result()
        return result is not None

    # ---- data logging ------------------------------------------------------

    def _sample_data(self):
        act = self._get_current_pose()
        if act is None:
            return
        theo = self._calc_theoretic_pose()
        err_x = act[0] - theo[0]
        err_y = act[1] - theo[1]
        err_yaw = normalize_angle(act[2] - theo[2])
        seg_t = time.time() - self._current_segment_start_time

        self._csv_logger.add_row([
            time.time(), f"{seg_t:.3f}",
            f"{act[0]:.4f}", f"{act[1]:.4f}", f"{act[2]:.4f}",
            f"{theo[0]:.4f}", f"{theo[1]:.4f}", f"{theo[2]:.4f}",
            f"{err_x:.4f}", f"{err_y:.4f}", f"{err_yaw:.4f}",
        ])
        self.get_logger().info(
            f"[Sample @ {seg_t:.1f}s] err_x:{err_x:.3f} err_y:{err_y:.3f} err_yaw:{err_yaw:.3f}")

    def _calc_theoretic_pose(self):
        sx, sy, syaw = self._current_segment_start_pose
        gx, gy, gyaw = self._current_segment_goal_pose
        t = time.time() - self._current_segment_start_time

        dx = gx - sx
        dy = gy - sy
        dist = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx) if dist > 0.05 else gyaw

        d1 = normalize_angle(target_heading - syaw)
        t_rot1 = abs(d1) / self.max_w if self.max_w > 0 else 0.0
        t_trans = dist / self.max_v if self.max_v > 0 else 0.0
        d2 = normalize_angle(gyaw - target_heading)
        t_rot2 = abs(d2) / self.max_w if self.max_w > 0 else 0.0

        if t <= t_rot1:
            p = t / t_rot1 if t_rot1 > 0 else 1.0
            return (sx, sy, syaw + d1 * p)
        elif t <= t_rot1 + t_trans:
            p = (t - t_rot1) / t_trans if t_trans > 0 else 1.0
            return (sx + dx * p, sy + dy * p, target_heading)
        elif t <= t_rot1 + t_trans + t_rot2:
            p = (t - t_rot1 - t_trans) / t_rot2 if t_rot2 > 0 else 1.0
            return (gx, gy, target_heading + d2 * p)
        else:
            return (gx, gy, gyaw)


def main():
    rclpy.init()
    node = MeasureTrajectoryError()
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
