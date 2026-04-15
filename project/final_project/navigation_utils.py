"""
navigation_utils.py
===================
Nav2 navigation utilities for the Semantic Fetch pipeline.

Uses HelloNode's existing spin thread via direct action-client callbacks
and threading.Event.  Does NOT call rclpy.spin_until_future_complete,
which conflicts with HelloNode's background spin thread.
"""

import math
import time
import threading

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def build_pose(node, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
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
    Thin Nav2 navigator that works alongside HelloNode's spin thread.

    Uses a direct NavigateToPose action client with async callbacks so
    that HelloNode's existing SingleThreadedExecutor drives everything —
    no extra executors, no rclpy.spin_until_future_complete.

    Usage:
        nav = Navigator(node)          # pass the HelloNode instance
        nav.wait_until_ready()
        result = nav.go_to(x, y, yaw)
        nav.shutdown()
    """

    TIMEOUT_SEC = 120.0

    def __init__(self, node=None):
        """
        node : HelloNode — provides the spin thread + clock.
               If None, falls back to BasicNavigator (standalone scripts).
        """
        self._node = node
        self._use_node = node is not None

        if self._use_node:
            self._client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
            self._done   = threading.Event()
            self._result = None
        else:
            from stretch_nav2.robot_navigator import BasicNavigator
            self._nav = BasicNavigator()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def wait_until_ready(self):
        if self._use_node:
            print('[NAV] Waiting for navigate_to_pose action server ...')
            self._client.wait_for_server()
            print('[NAV] Nav2 is ready for use!')
        else:
            self._nav.waitUntilNav2Active()

    def shutdown(self):
        if not self._use_node:
            self._nav.destroy_node()

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def go_to(self, x: float, y: float, yaw: float,
              timeout_sec: float = None) -> str:
        """
        Navigate to (x, y, yaw) in the map frame.
        Blocks until complete, failed, or timeout.
        Returns NavResult.SUCCEEDED / FAILED / CANCELED.
        """
        timeout_sec = timeout_sec or self.TIMEOUT_SEC

        if self._use_node:
            return self._go_callback(x, y, yaw, timeout_sec)
        else:
            return self._go_basic(x, y, yaw, timeout_sec)

    # ------------------------------------------------------------------
    # Callback-based implementation (HelloNode path)
    # ------------------------------------------------------------------

    def _go_callback(self, x, y, yaw, timeout_sec):
        pose = build_pose(self._node, x, y, yaw)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._done.clear()
        self._result = None

        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

        if not self._done.wait(timeout=timeout_sec):
            print(f'[NAV] Navigation timed out after {timeout_sec:.0f}s.')
            return NavResult.CANCELED

        return self._result

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('[NAV] Goal rejected by Nav2.')
            self._result = NavResult.FAILED
            self._done.set()
            return
        print('[NAV] Goal accepted, navigating ...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._result = NavResult.SUCCEEDED
            print('[NAV] Arrived at goal.')
        elif status == GoalStatus.STATUS_CANCELED:
            self._result = NavResult.CANCELED
            print('[NAV] Navigation canceled.')
        else:
            self._result = NavResult.FAILED
            print(f'[NAV] Navigation failed (status={status}).')
        self._done.set()

    # ------------------------------------------------------------------
    # BasicNavigator fallback (standalone scripts, no HelloNode)
    # ------------------------------------------------------------------

    def _go_basic(self, x, y, yaw, timeout_sec):
        from rclpy.duration import Duration
        pose = build_pose(self._nav, x, y, yaw)
        self._nav.goToPose(pose)
        start = self._nav.get_clock().now()
        while True:
            try:
                done = self._nav.isTaskComplete()
            except (IndexError, ValueError):
                time.sleep(0.05)
                continue
            if done:
                break
            if self._nav.get_clock().now() - start > Duration(seconds=timeout_sec):
                self._nav.cancelTask()
                return NavResult.CANCELED
            time.sleep(0.05)
        from stretch_nav2.robot_navigator import TaskResult
        r = self._nav.getResult()
        if r == TaskResult.SUCCEEDED:
            return NavResult.SUCCEEDED
        if r == TaskResult.CANCELED:
            return NavResult.CANCELED
        return NavResult.FAILED
