"""
search_behavior.py
==================
360-degree spin search for a named object within a semantic zone.

The robot arrives at the zone nav_point, fixes the head forward and slightly
downward, then rotates the base in SPIN_STEPS increments to cover a full 360°.
YOLO-E is queried at each stop. On first detection, the 3-D centroid of the
mask is projected into the base_link frame and returned.
If nothing is found after the full spin, the function returns None.

Returns:
    PoseStamped in base_link frame, or None if the object is not found.

Usage (from fetch.py):
    from search_behavior import SearchBehavior
    searcher = SearchBehavior(node, nav, obj_name)
    goal_pose = searcher.search(zone)
"""

import time
import threading
import os.path as osp

import cv2
import numpy as np
import rclpy
import tf2_ros
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mm2026', 'lab3'))
import detection_utils

from navigation_utils import Navigator, NavResult


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

MODEL_PATH = osp.join('/home/hello-robot/models', 'yoloe-26s-seg.pt')

# Head camera topics (Part 2 / head camera from lab 3)
COLOR_TOPIC    = '/camera/color/image_raw'
DEPTH_TOPIC    = '/camera/aligned_depth_to_color/image_raw'
CAM_INFO_TOPIC = '/camera/color/camera_info'

# Head joint limits (radians) – Stretch 3 SE3
HEAD_PAN_MIN  = -3.9   # full left
HEAD_PAN_MAX  =  1.5   # full right (toward arm side)
HEAD_TILT     = -0.6   # looking slightly downward at table height

CONF_THRESHOLD = 0.25  # YOLO-E confidence threshold (match lab3)
MIN_DEPTH_MM   = 200   # ignore depth values below this (noise)
MAX_DEPTH_MM   = 3000  # ignore depth values above this


# ---------------------------------------------------------------------------
# SearchBehavior
# ---------------------------------------------------------------------------

class SearchBehavior:
    """
    Attaches camera subscribers to an existing HelloNode and uses it to
    sweep the head while running YOLO-E detection on head-camera frames.

    Parameters
    ----------
    node : HelloNode
        The running HelloNode instance (provides move_to_pose, TF, spin).
    nav  : Navigator
        The running Navigator instance (reserved for future use).
    obj_name : str
        Plain-text name passed to YOLO-E's set_classes(), e.g. "water bottle".
    """

    PHASE1_STEPS = 6   # head sweep steps (HEAD_PAN_MIN → HEAD_PAN_MAX)

    # Seconds to wait for a fresh camera frame after rotating
    FRAME_WAIT_SEC = 1.5

    # Seconds to pause after rotating base (camera stabilisation)
    SETTLE_SEC = 0.5

    def __init__(self, node, nav: Navigator, obj_name: str):
        self.node     = node
        self.nav      = nav
        self.obj_name = obj_name

        # --- YOLO-E -------------------------------------------------------
        self.model = YOLO(MODEL_PATH)
        self.model.set_classes([obj_name])

        # --- Camera state -------------------------------------------------
        self.bridge      = CvBridge()
        self._frame_lock = threading.Lock()
        self._new_frame  = threading.Event()
        self.latest_color    = None
        self.latest_depth    = None
        self.latest_cam_info = None

        # --- Debug image publisher ----------------------------------------
        self._debug_pub = node.create_publisher(Image, '/search/debug_image', 10)

        # Register camera subscribers on the existing node
        self._color_sub = message_filters.Subscriber(node, Image, COLOR_TOPIC)
        self._depth_sub = message_filters.Subscriber(node, Image, DEPTH_TOPIC)
        self._info_sub  = message_filters.Subscriber(node, CameraInfo, CAM_INFO_TOPIC)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub, self._info_sub],
            queue_size=10,
            slop=0.05,
        )
        self._sync.registerCallback(self._image_callback)

        # --- TF -----------------------------------------------------------
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)

    # ------------------------------------------------------------------
    # Camera callback
    # ------------------------------------------------------------------

    def _image_callback(self, color_msg, depth_msg, cam_info_msg):
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            # Head camera on Stretch is mounted sideways → rotate upright
            color = cv2.rotate(color, cv2.ROTATE_90_CLOCKWISE)
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
            depth = cv2.rotate(depth, cv2.ROTATE_90_CLOCKWISE)
        except CvBridgeError as e:
            self.node.get_logger().warn(f'[SEARCH] CvBridge error: {e}')
            return

        with self._frame_lock:
            self.latest_color    = color
            self.latest_depth    = depth
            self.latest_cam_info = cam_info_msg
            self._new_frame.set()

    # ------------------------------------------------------------------
    # Detection helpers
    # ------------------------------------------------------------------

    def _wait_for_fresh_frame(self) -> bool:
        """Block until a new camera frame arrives or timeout."""
        self._new_frame.clear()
        return self._new_frame.wait(timeout=self.FRAME_WAIT_SEC)

    def _detect_current_frame(self) -> 'PoseStamped | None':
        """
        Run YOLO-E on the latest frame and return a PoseStamped in
        base_link if the object is detected, else None.
        """
        with self._frame_lock:
            if self.latest_color is None:
                return None
            color    = self.latest_color.copy()
            depth    = self.latest_depth.copy()
            cam_info = self.latest_cam_info

        # YOLO-E inference (RGB expected)
        results    = self.model(color, conf=CONF_THRESHOLD)  # pass BGR directly (match lab3)
        detections = detection_utils.parse_results(results)

        # Show live detection visualization (same as lab3)
        detection_utils.visualize_detections_masks(
            part=2, detections=detections, rgb_image=color, depth_image=depth
        )

        if not detections:
            return None

        # Use the highest-confidence detection
        target = detections[0]
        mask   = target['mask']   # Nx2 polygon in pixel coords

        goal_xyz = self._mask_to_3d_centroid(mask, depth, cam_info)
        if goal_xyz is None:
            return None

        goal_pose_cam = detection_utils.get_pose_msg(
            cam_info.header.stamp,
            cam_info.header.frame_id,
            goal_xyz,
        )

        # Transform from camera frame → base_link
        try:
            goal_pose_base = self.tf_buffer.transform(goal_pose_cam, 'base_link')
            return goal_pose_base
        except Exception as e:
            self.node.get_logger().warn(f'[SEARCH] TF transform failed: {e}')
            return None

    def _mask_to_3d_centroid(self, mask, depth, cam_info):
        """
        Rasterize the mask polygon, project all interior pixels to 3-D using
        the depth image, and return the centroid (lab 3 Part 2 approach).
        Returns None if too few valid depth points are found.
        """
        h, w = depth.shape[:2]
        mask_img = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_img, [mask], 1)
        ys, xs = np.where(mask_img > 0)

        if len(xs) == 0:
            return None

        points_3d = []
        for x, y in zip(xs, ys):
            z_mm = float(depth[y, x])
            if z_mm < MIN_DEPTH_MM or z_mm > MAX_DEPTH_MM or np.isnan(z_mm):
                continue
            xyz = detection_utils.pixel_to_3d((x, y), z_mm, cam_info)
            if not np.any(np.isnan(xyz)):
                points_3d.append(xyz)

        if len(points_3d) < 10:   # require at least 10 valid points
            return None

        return np.mean(points_3d, axis=0)

    # ------------------------------------------------------------------
    # Head sweep search
    # ------------------------------------------------------------------

    def _head_sweep(self, n_steps: int) -> 'PoseStamped | None':
        """
        Sweep joint_head_pan from HEAD_PAN_MIN to HEAD_PAN_MAX in n_steps.
        Runs YOLO-E at each stop.
        Returns PoseStamped on first detection, or None.
        """
        pan_angles = np.linspace(HEAD_PAN_MIN, HEAD_PAN_MAX, n_steps)

        for pan in pan_angles:
            self.node.move_to_pose(
                {'joint_head_pan': float(pan), 'joint_head_tilt': HEAD_TILT},
                blocking=True,
            )
            time.sleep(self.SETTLE_SEC)

            if not self._wait_for_fresh_frame():
                self.node.get_logger().warn(
                    f'[SEARCH] No camera frame at pan={np.degrees(pan):.1f}°, skipping.'
                )
                continue

            pose = self._detect_current_frame()
            if pose is not None:
                print(f'[SEARCH] "{self.obj_name}" detected at head_pan={np.degrees(pan):.1f}°')
                return pose

        return None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def search(self, zone: dict) -> 'PoseStamped | None':
        """
        Sweep head pan across full range looking for the object.
        Assumes the robot is already at zone['nav_point'].

        Parameters
        ----------
        zone : dict
            A zone config dict from semantic_map.yaml.

        Returns
        -------
        PoseStamped in base_link frame, or None if not found.
        """
        print(f'[SEARCH] Sweeping head ({self.PHASE1_STEPS} steps) to find "{self.obj_name}" ...')
        pose = self._head_sweep(self.PHASE1_STEPS)
        self._reset_head()

        if pose is None:
            print(f'[SEARCH] "{self.obj_name}" not found after full head sweep.')

        return pose

    def _reset_head(self):
        """Return head to forward-looking pose after search."""
        self.node.move_to_pose(
            {'joint_head_pan': -1.6, 'joint_head_tilt': -0.5},
            blocking=True,
        )
