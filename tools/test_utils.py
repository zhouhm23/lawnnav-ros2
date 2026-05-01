#!/usr/bin/env python3
"""
Shared utility module for test scripts.
Provides math helpers, CSV logging, stuck/collision detection,
closed-loop rotation, cmd_vel monitoring, and localization helpers.
"""

import math
import time
import csv
import os
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist


# ---------------------------------------------------------------------------
# Pure math helpers
# ---------------------------------------------------------------------------

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to 2D yaw angle (radians)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Build a geometry_msgs/Quaternion from a yaw angle."""
    half = yaw * 0.5
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half)
    quat.w = math.cos(half)
    return quat


def make_pose_stamped(x: float, y: float, yaw: float,
                      frame_id: str = "map") -> PoseStamped:
    """Build a stamped pose in the given frame (default 'map')."""
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    msg.pose.orientation = quaternion_from_yaw(yaw)
    return msg


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# CSVLogger
# ---------------------------------------------------------------------------

class CSVLogger:
    """Standardised CSV writer that auto-generates timestamped filenames
    and flushes every row so data survives unexpected crashes."""

    def __init__(self, directory: str, prefix: str, headers: list):
        os.makedirs(directory, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(directory, f"{prefix}_{timestamp}.csv")
        self._file = open(self.filepath, 'w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(headers)
        self._file.flush()

    def add_row(self, row: list) -> None:
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        self._file.close()


# ---------------------------------------------------------------------------
# StuckDetector
# ---------------------------------------------------------------------------

class StuckDetector:
    """Detects whether the robot is physically stuck by tracking whether
    position / orientation have changed meaningfully over a sliding window.

    Usage
    -----
        sd = StuckDetector(window_sec=5.0, linear_threshold=0.05,
                           angular_threshold=0.05)
        # in your main loop, after obtaining (x, y, yaw):
        sd.update(x, y, yaw)
        if sd.is_stuck():
            print("Robot is stuck!")
    """

    def __init__(self, window_sec: float = 5.0,
                 linear_threshold: float = 0.05,
                 angular_threshold: float = 0.05):
        self._window = window_sec
        self._linear_threshold = linear_threshold
        self._angular_threshold = angular_threshold
        self._history: deque = deque()  # (timestamp, x, y, yaw)

    def update(self, x: float, y: float, yaw: float) -> None:
        now = time.time()
        self._history.append((now, x, y, yaw))
        # Prune old entries
        cutoff = now - self._window
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()

    def is_stuck(self) -> bool:
        """True if displacement AND angular change over the window are
        both below their thresholds."""
        if len(self._history) < 2:
            return False
        now = time.time()
        # Collect entries still within the window
        recent = [(t, x, y, yaw)
                  for t, x, y, yaw in self._history
                  if now - t <= self._window]
        if len(recent) < 2:
            return False
        t0, x0, y0, yaw0 = recent[0]
        _, x1, y1, yaw1 = recent[-1]
        dist = math.hypot(x1 - x0, y1 - y0)
        angle_delta = abs(normalize_angle(yaw1 - yaw0))
        return dist < self._linear_threshold and angle_delta < self._angular_threshold

    def reset(self) -> None:
        self._history.clear()


# ---------------------------------------------------------------------------
# CmdVelMonitor
# ---------------------------------------------------------------------------

class CmdVelMonitor:
    """Lightweight monitor that tracks whether /cmd_vel has recently
    contained a non-zero motion command.

    Two modes
    --------
    1. Automatic — pass *node* to __init__; a /cmd_vel subscription is
       created for you.
    2. Manual — call :meth:`feed` yourself with the latest linear.x and
       angular.z values.
    """

    def __init__(self, node=None, active_window: float = 1.0):
        self._active_window = active_window
        self._last_nonzero_time = 0.0
        self._sub = None
        if node is not None:
            self._sub = node.create_subscription(
                Twist, '/cmd_vel', self._cmd_vel_callback, 10)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self.feed(msg.linear.x, msg.angular.z)

    def feed(self, linear_x: float, angular_z: float) -> None:
        if abs(linear_x) > 0.001 or abs(angular_z) > 0.001:
            self._last_nonzero_time = time.time()

    def is_active(self) -> bool:
        """True if a non-zero command was seen within the active window."""
        return (time.time() - self._last_nonzero_time) < self._active_window

    def destroy(self) -> None:
        if self._sub is not None:
            self._sub.destroy()


# ---------------------------------------------------------------------------
# rotate_360 — closed-loop, pose-feedback rotation
# ---------------------------------------------------------------------------

def rotate_360(node,
               cmd_vel_pub,
               get_yaw_fn,
               angular_speed: float = 0.5,
               timeout: float = 30.0) -> bool:
    """Rotate the robot one full turn.

    Strategy (the user's insight, simplest & most robust):
    The normalised yaw difference  |normalise(current − start)|
    naturally goes  0° → 180° → 0°  during one full revolution.
    Phase 1 — wait for the difference to exceed 90° (proves we're turning).
    Phase 2 — wait for it to drop back below 15° (back where we started).
    Stop immediately — no cumulative tracking, no PID, no timing guesswork.
    """
    logger = node.get_logger()

    # --- wait for initial yaw ---------------------------------------------
    start_yaw = None
    deadline = time.time() + 5.0
    while start_yaw is None and time.time() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        start_yaw = get_yaw_fn()
    if start_yaw is None:
        logger.error("rotate_360: no localization after 5 s — cannot rotate")
        return False

    # --- rotate until yaw goes far, then returns --------------------------
    twist = Twist()
    twist.angular.z = float(angular_speed)
    current_yaw = start_yaw
    start_time = time.time()

    GONE_FAR_THRESHOLD = math.radians(90)     # must cross this first
    BACK_HOME_THRESHOLD = math.radians(15)    # stop when close to start

    phase = "going"  # waiting for yaw diff to exceed GONE_FAR_THRESHOLD

    dir_str = "CCW" if angular_speed > 0 else "CW"
    logger.info(
        f"rotate_360: {dir_str}, stopping when yaw returns to start "
        f"(cmd {abs(angular_speed):.2f} rad/s)"
    )

    while rclpy.ok():
        cmd_vel_pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.1)

        yaw = get_yaw_fn()
        if yaw is not None:
            current_yaw = yaw

        yaw_diff = abs(normalize_angle(current_yaw - start_yaw))
        elapsed = time.time() - start_time

        if phase == "going":
            if yaw_diff > GONE_FAR_THRESHOLD:
                phase = "returning"
                logger.info(
                    f"rotate_360: turned {math.degrees(yaw_diff):.0f}° — "
                    f"waiting to come back"
                )
        else:  # returning
            if yaw_diff <= BACK_HOME_THRESHOLD:
                logger.info(
                    f"rotate_360: back home — "
                    f"yaw diff {math.degrees(yaw_diff):.1f}° "
                    f"in {elapsed:.1f} s"
                )
                break

        if elapsed > timeout:
            logger.error(
                f"rotate_360: TIMEOUT after {elapsed:.1f} s — "
                f"yaw diff {math.degrees(yaw_diff):.1f}°"
            )
            break

    # --- stop -------------------------------------------------------------
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    return phase == "returning" and yaw_diff <= BACK_HOME_THRESHOLD


# ---------------------------------------------------------------------------
# wait_for_localization
# ---------------------------------------------------------------------------

def wait_for_localization(node,
                          topic: str = "/localization_pose",
                          timeout: float = 30.0) -> bool:
    """Spin until a message arrives on *topic* or *timeout* expires.

    Returns True if a message was received."""
    from geometry_msgs.msg import PoseWithCovarianceStamped

    latest = None

    def _cb(msg):
        nonlocal latest
        latest = msg

    sub = node.create_subscription(PoseWithCovarianceStamped, topic, _cb, 10)
    start = time.time()
    while latest is None and rclpy.ok() and (time.time() - start) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if latest is not None:
        node.get_logger().info(f"wait_for_localization: received pose on {topic}")
        return True
    node.get_logger().error(f"wait_for_localization: no message on {topic} after {timeout} s")
    return False
