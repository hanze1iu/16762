"""
grasp_pipeline.py
=================
M4 — approach and grasp an object given its 3D pose in base_link.

Wraps lab 3 IK logic (ik_ros_utils.py) into a single blocking `grasp()` call
that the fetch.py orchestrator can call after SearchBehavior returns a goal pose.

Usage (from fetch.py):
    from grasp_pipeline import GraspPipeline
    grasper = GraspPipeline(node)
    success = grasper.grasp(goal_pose)   # goal_pose: PoseStamped in base_link
"""

import sys
import os
import time
import threading

import numpy as np
import ikpy.utils.geometry
import rclpy
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# ik_ros_utils lives in the student lab3 directory (has full implementation)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lab3'))
import ik_ros_utils as ik


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Mirror of lab3 READY_POSE_P2 — lift up, arm retracted, gripper open, head forward
READY_POSE = {
    'joint_lift':        0.8,
    'joint_arm':         0.0,
    'joint_wrist_yaw':   0.5,
    'joint_wrist_pitch': -0.1,
    'gripper_aperture':  0.8,
}
READY_HEAD = {
    'joint_head_pan':  -1.7,
    'joint_head_tilt': -0.5,
}

GRIPPER_FRAME  = 'link_grasp_center'
BASE_FRAME     = 'base_link'

DELTA          = 0.01   # close gripper when within this distance (metres)
SAFETY_X       = -0.03   # stand-off: approach from slightly in front of centroid
MAX_STEPS      = 40     # iterations before giving up
SETTLE_SEC     = 0.4    # wait after each move for TF to update
JOINT_TIMEOUT  = 5.0    # seconds to wait for first joint-state message

# No static calibration offsets — live detection self-corrects (lab3 style).
# safety_x standoff is applied inside _compute_waypoint (same as lab3).
GRASP_X_OFFSET = -0.01
GRASP_Y_OFFSET =  0.0
GRASP_Z_OFFSET = 0.0


# ---------------------------------------------------------------------------
# GraspPipeline
# ---------------------------------------------------------------------------

class GraspPipeline:
    """
    Iteratively moves the arm toward a 3D goal in base_link, then closes the
    gripper when the gripper centre is within DELTA of the target.

    Parameters
    ----------
    node : HelloNode (FetchNode)
        The running HelloNode — provides move_to_pose() and the ROS2 executor.
    """

    def __init__(self, node):
        self.node = node

        # Joint state tracking
        self._joint_lock  = threading.Lock()
        self._joint_state = {}   # {joint_name: position}

        self._joint_sub = node.create_subscription(
            JointState,
            '/stretch/joint_states',
            self._joint_cb,
            1,
        )

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------

    def _joint_cb(self, msg: JointState):
        names_needed = {
            'joint_lift', 'joint_arm_l0',
            'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
        }
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in names_needed:
                    self._joint_state[name] = pos

    def _wait_for_joint_states(self) -> bool:
        """Block until all 5 required joints have been received."""
        deadline = time.time() + JOINT_TIMEOUT
        while time.time() < deadline:
            with self._joint_lock:
                if len(self._joint_state) >= 5:
                    return True
            time.sleep(0.05)
        return False

    def _get_joint_state_copy(self) -> dict:
        with self._joint_lock:
            return dict(self._joint_state)

    # ------------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------------

    def _get_gripper_pos(self):
        """Return gripper position as np.array([x,y,z]) in base_link, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, GRIPPER_FRAME, rclpy.time.Time()
            )
            return ik.get_xyz_from_msg(tf)
        except Exception as e:
            self.node.get_logger().warn(f'[GRASP] TF error: {e}')
            return None

    # ------------------------------------------------------------------
    # Waypoint computation (identical to lab3 grasp_objects logic)
    # ------------------------------------------------------------------

    def _compute_waypoint(self, goal_pos: np.ndarray, gripper_pos: np.ndarray):
        """
        If the gripper is more than DELTA away from the safe_goal, step
        DELTA toward it.  Otherwise go directly to safe_goal.
        """
        safe_goal = goal_pos.copy()
        safe_goal[0] += SAFETY_X   # stand-off along x (forward)

        diff = safe_goal - gripper_pos
        dist = np.linalg.norm(diff)

        # Use only y,z distance for close-gripper trigger.
        # x is ignored because the base cannot reliably move backward far enough.
        dist_yz = np.linalg.norm((safe_goal - gripper_pos)[[1, 2]])

        if dist > DELTA:
            waypoint = gripper_pos + (diff / dist) * DELTA
        else:
            waypoint = safe_goal

        # Never approach from below
        waypoint[2] = max(waypoint[2], safe_goal[2])

        orient = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
        return waypoint, orient, dist_yz

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def grasp(self, goal_pose: PoseStamped, get_goal_fn=None) -> bool:
        """
        Move to the ready pose, then iteratively approach goal_pose and
        close the gripper.

        Parameters
        ----------
        goal_pose : PoseStamped
            Initial 3D centroid of the target object in base_link frame.
        get_goal_fn : callable, optional
            If provided, called at each IK step to get the latest PoseStamped
            in base_link. Enables live correction (lab3-style closed-loop).

        Returns
        -------
        bool — True if gripper was closed on the object, False on failure.
        """
        def _pose_to_goal_pos(pose: PoseStamped) -> np.ndarray:
            return np.array([
                pose.pose.position.x + GRASP_X_OFFSET,
                pose.pose.position.y + GRASP_Y_OFFSET,
                pose.pose.position.z + GRASP_Z_OFFSET,
            ])

        goal_pos = _pose_to_goal_pos(goal_pose)
        print(f'[GRASP] Initial target in base_link: x={goal_pos[0]:.3f}  '
              f'y={goal_pos[1]:.3f}  z={goal_pos[2]:.3f}')

        # ---- 1. Move to ready pose ----
        print('[GRASP] Moving to ready pose ...')
        self.node.move_to_pose(READY_POSE, blocking=True)
        self.node.move_to_pose(READY_HEAD, blocking=True)

        # ---- 2. Wait for joint states ----
        if not self._wait_for_joint_states():
            print('[GRASP] ERROR: timed out waiting for joint states.')
            return False

        # ---- 3. Iterative approach ----
        print(f'[GRASP] Starting iterative approach (max {MAX_STEPS} steps) ...')

        prev_dist   = float('inf')
        stuck_count = 0
        STUCK_THRESHOLD = 0.005   # less than 5mm progress = stuck
        STUCK_STEPS     = 2       # force close after this many stuck steps

        LIVE_UPDATE_DIST = 0.20   # freeze live updates when closer than this (z unreliable)

        for step in range(MAX_STEPS):
            # Refresh goal from live detector only when far away.
            # When close (< LIVE_UPDATE_DIST), z becomes unreliable due to arm
            # occlusion and transparent surface depth failure — freeze target.
            if get_goal_fn is not None and prev_dist > LIVE_UPDATE_DIST:
                fresh = get_goal_fn()
                if fresh is not None:
                    goal_pos = _pose_to_goal_pos(fresh)

            gripper_pos = self._get_gripper_pos()
            if gripper_pos is None:
                print('[GRASP] ERROR: cannot get gripper TF.')
                return False

            waypoint, orient, dist = self._compute_waypoint(goal_pos, gripper_pos)
            print(f'[GRASP] Step {step+1:02d}: dist={dist:.3f} m  '
                  f'target=({goal_pos[0]:.3f},{goal_pos[1]:.3f},{goal_pos[2]:.3f})  '
                  f'gripper=({gripper_pos[0]:.3f},{gripper_pos[1]:.3f},{gripper_pos[2]:.3f})')

            # Stuck detection — force close if no progress for STUCK_STEPS consecutive steps
            if prev_dist - dist < STUCK_THRESHOLD:
                stuck_count += 1
            else:
                stuck_count = 0
            prev_dist = dist

            if stuck_count >= STUCK_STEPS:
                print(f'[GRASP] Arm stuck for {STUCK_STEPS} steps — forcing gripper close.')
                self.node.move_to_pose({'gripper_aperture': -0.2}, blocking=True)
                print('[GRASP] Gripper closed. Retracting arm ...')
                self.node.move_to_pose({'joint_arm': 0.0}, blocking=True)
                print('[GRASP] Arm retracted.')
                return True

            if dist <= DELTA:
                print('[GRASP] Within reach — closing gripper.')
                self.node.move_to_pose({'gripper_aperture': -0.2}, blocking=True)
                print('[GRASP] Gripper closed. Retracting arm ...')
                # Retract arm only (no lift change) to avoid dragging object across table.
                self.node.move_to_pose({'joint_arm': 0.0}, blocking=True)
                print('[GRASP] Arm retracted.')
                return True

            # IK solve
            js     = self._get_joint_state_copy()
            q_init = ik.get_current_configuration(js)
            q_soln = ik.get_grasp_goal(waypoint, orient, q_init)

            if q_soln is None:
                print(f'[GRASP] IK has no solution at step {step+1}. Aborting.')
                return False

            ik.move_to_configuration(self.node, q_soln)
            time.sleep(SETTLE_SEC)

        print(f'[GRASP] Reached max steps ({MAX_STEPS}) without grasping.')
        return False
