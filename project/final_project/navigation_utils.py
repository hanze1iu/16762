"""
navigation_utils.py
===================
Thin wrapper around Nav2 BasicNavigator for the Semantic Fetch pipeline.

Provides:
  - Navigator      : main class, handles Nav2 lifecycle
  - build_pose()   : construct a map-frame PoseStamped from (x, y, yaw)
  - NavResult      : enum-like result codes
"""

import math
import time
from geometry_msgs.msg import PoseStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def build_pose(navigator, x: float, y: float, yaw: float) -> PoseStamped:
    """
    Build a map-frame PoseStamped from (x, y, yaw_radians).
    Reused from lab 4 make_pose() pattern.
    """
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class NavResult:
    SUCCEEDED = 'succeeded'
    FAILED    = 'failed'
    CANCELED  = 'canceled'


# ---------------------------------------------------------------------------
# Navigator
# ---------------------------------------------------------------------------

class Navigator:
    """
    Wraps BasicNavigator with convenience methods used by the fetch pipeline.

    Usage:
        nav = Navigator()
        nav.wait_until_ready()
        nav.go_to(x, y, yaw)
        nav.shutdown()
    """

    TIMEOUT_SEC = 120.0   # max seconds to wait for a single nav goal

    def __init__(self):
        self._nav = BasicNavigator()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0):
        """
        Tell Nav2 where the robot starts in the map.
        Call this once before wait_until_ready() if AMCL hasn't already
        received a pose estimate from RViz.
        """
        pose = build_pose(self._nav, x, y, yaw)
        self._nav.setInitialPose(pose)

    def wait_until_ready(self):
        """Block until Nav2 is fully active."""
        self._nav.waitUntilNav2Active()

    def shutdown(self):
        self._nav.destroy_node()

    # ------------------------------------------------------------------
    # Single goal
    # ------------------------------------------------------------------

    def go_to(self, x: float, y: float, yaw: float,
              timeout_sec: float = None) -> str:
        """
        Navigate to (x, y, yaw) in the map frame.
        Blocks until complete or timeout.
        Returns a NavResult string.
        """
        timeout_sec = timeout_sec or self.TIMEOUT_SEC
        pose = build_pose(self._nav, x, y, yaw)

        self._nav.goToPose(pose)
        start = self._nav.get_clock().now()

        while True:
            try:
                done = self._nav.isTaskComplete()
            except IndexError:
                time.sleep(0.05)
                continue
            if done:
                break
            elapsed = self._nav.get_clock().now() - start
            if elapsed > Duration(seconds=timeout_sec):
                self._nav.cancelTask()
                print(f'[NAV] Timed out after {timeout_sec}s, cancelling.')
                return NavResult.CANCELED
            time.sleep(0.05)

        return self._task_result_to_str(self._nav.getResult())

    # ------------------------------------------------------------------
    # Waypoint sequence
    # ------------------------------------------------------------------

    def follow_waypoints(self, waypoints: list, timeout_sec: float = None) -> str:
        """
        Navigate through a list of (x, y, yaw) tuples in order.
        Blocks until all waypoints are reached or timeout.
        Returns a NavResult string.

        waypoints: list of dicts with keys 'x', 'y', 'yaw'
                   OR list of (x, y, yaw) tuples
        """
        timeout_sec = timeout_sec or self.TIMEOUT_SEC * len(waypoints)
        poses = []
        for wp in waypoints:
            if isinstance(wp, dict):
                poses.append(build_pose(self._nav, wp['x'], wp['y'], wp.get('yaw', 0.0)))
            else:
                poses.append(build_pose(self._nav, wp[0], wp[1], wp[2]))

        self._nav.followWaypoints(poses)
        start = self._nav.get_clock().now()

        i = 0
        while True:
            try:
                done = self._nav.isTaskComplete()
            except IndexError:
                time.sleep(0.05)
                continue
            if done:
                break
            elapsed = self._nav.get_clock().now() - start
            if elapsed > Duration(seconds=timeout_sec):
                self._nav.cancelTask()
                print('[NAV] Waypoint sequence timed out, cancelling.')
                return NavResult.CANCELED

            feedback = self._nav.getFeedback()
            if feedback:
                wp_idx = feedback.current_waypoint + 1
                if wp_idx != i:
                    i = wp_idx
                    print(f'[NAV] Reaching waypoint {i}/{len(poses)} ...')
            time.sleep(0.05)

        return self._task_result_to_str(self._nav.getResult())

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _task_result_to_str(self, result) -> str:
        if result == TaskResult.SUCCEEDED:
            return NavResult.SUCCEEDED
        if result == TaskResult.CANCELED:
            return NavResult.CANCELED
        return NavResult.FAILED
