"""
search_behavior.py
==================
Head-sweep search. SearchBehavior extends Node (same as lab3 YOLOEObjectDetector)
and spins on its own thread so camera callbacks are guaranteed to fire.
"""

import time
import threading
import os.path as osp

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
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
# Constants  (match lab3)
# ---------------------------------------------------------------------------

MODEL_PATH     = '/home/hello-robot/models/yoloe-26s-seg.pt'
COLOR_TOPIC    = '/camera/color/image_raw'
DEPTH_TOPIC    = '/camera/aligned_depth_to_color/image_raw'
CAM_INFO_TOPIC = '/camera/color/camera_info'

HEAD_PAN_MIN   = -3.9
HEAD_PAN_MAX   =  1.5
HEAD_TILT      = -0.6
CONF_THRESHOLD =  0.25
MIN_DEPTH_MM   =  200
MAX_DEPTH_MM   =  3000
PHASE1_STEPS   =  6
SETTLE_SEC     =  0.8
FRAME_WAIT_SEC =  3.0


# ---------------------------------------------------------------------------
# SearchBehavior — extends Node, same architecture as lab3
# ---------------------------------------------------------------------------

class SearchBehavior(Node):

    def __init__(self, hello_node, nav: Navigator, obj_name: str):
        # Own Node with unique name — has its own executor + spin
        super().__init__('search_behavior_node')

        self.hello_node = hello_node   # for move_to_pose / arm control
        self.nav        = nav
        self.obj_name   = obj_name

        # YOLO-E  — same as lab3
        self.model = YOLO(MODEL_PATH)
        self.model.set_classes([obj_name])

        # Camera state  — same as lab3
        self.bridge                = CvBridge()
        self.latest_color          = None
        self.latest_depth          = None
        self.latest_color_cam_info = None
        self._frame_lock           = threading.Lock()
        self._new_frame            = threading.Event()

        # Subscribers + synchroniser  — identical to lab3
        self.color_sub = message_filters.Subscriber(self, Image, COLOR_TOPIC)
        self.depth_sub = message_filters.Subscriber(self, Image, DEPTH_TOPIC)
        self.color_cam_info_sub = message_filters.Subscriber(self, CameraInfo, CAM_INFO_TOPIC)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.color_cam_info_sub],
            queue_size=10,
            slop=0.05,
        )
        self.synchronizer.registerCallback(self.image_callback)

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Spin on own thread — same idea as rclpy.spin(node) in lab3
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()

        print('[SEARCH] SearchBehavior node started, waiting for camera frames...')

    # ------------------------------------------------------------------
    # Camera callback  — identical to lab3
    # ------------------------------------------------------------------

    def image_callback(self, color_msg, depth_msg, color_cam_info_msg):
        try:
            color = cv2.rotate(
                self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8'),
                cv2.ROTATE_90_CLOCKWISE,
            )
            depth = cv2.rotate(
                self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough'),
                cv2.ROTATE_90_CLOCKWISE,
            )
        except CvBridgeError as e:
            print(f'[SEARCH] CvBridge error: {e}')
            return

        with self._frame_lock:
            self.latest_color          = color
            self.latest_depth          = depth
            self.latest_color_cam_info = color_cam_info_msg
            self._new_frame.set()
        # NOTE: cv2.imshow must be called from main thread — visualize in search() instead

    # ------------------------------------------------------------------
    # Detection  — same as lab3 publish_goals_callback
    # ------------------------------------------------------------------

    def _detect(self) -> 'PoseStamped | None':
        with self._frame_lock:
            if self.latest_color is None:
                return None
            color    = self.latest_color.copy()
            depth    = self.latest_depth.copy()
            cam_info = self.latest_color_cam_info

        results    = self.model(color, conf=CONF_THRESHOLD)
        detections = detection_utils.parse_results(results)

        if not detections:
            return None

        target       = detections[0]
        mask_polygon = target['mask']

        # 3D centroid — lab3 Part 2
        h, w = depth.shape[:2]
        mask_bin = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask_bin, [mask_polygon], 1)
        ys, xs = np.where(mask_bin)

        points_3d = []
        for x, y in zip(xs, ys):
            z_mm = float(depth[y, x])
            if z_mm < MIN_DEPTH_MM or z_mm > MAX_DEPTH_MM or np.isnan(z_mm):
                continue
            xyz = detection_utils.pixel_to_3d((x, y), z_mm, cam_info)
            if not np.any(np.isnan(xyz)):
                points_3d.append(xyz)

        if len(points_3d) < 10:
            return None

        centroid = np.mean(points_3d, axis=0)
        pose_cam = detection_utils.get_pose_msg(
            cam_info.header.stamp,
            cam_info.header.frame_id,
            centroid,
        )

        try:
            return self.tf_buffer.transform(pose_cam, 'base_link')
        except Exception as e:
            print(f'[SEARCH] TF failed: {e}')
            return None

    # ------------------------------------------------------------------
    # Head sweep
    # ------------------------------------------------------------------

    def search(self, zone: dict) -> 'PoseStamped | None':
        print(f'[SEARCH] Sweeping head ({PHASE1_STEPS} steps) for "{self.obj_name}" ...')

        pan_angles = np.linspace(HEAD_PAN_MIN, HEAD_PAN_MAX, PHASE1_STEPS)

        for pan in pan_angles:
            self.hello_node.move_to_pose(
                {'joint_head_pan': float(pan), 'joint_head_tilt': HEAD_TILT},
                blocking=True,
            )
            time.sleep(SETTLE_SEC)

            self._new_frame.clear()
            got_frame = self._new_frame.wait(timeout=FRAME_WAIT_SEC)
            if not got_frame:
                print(f'[SEARCH] No frame at pan={np.degrees(pan):.0f}°, skipping.')
                continue

            # Visualize from main thread (cv2.imshow must be on main thread)
            with self._frame_lock:
                vis_color = self.latest_color.copy() if self.latest_color is not None else None
                vis_depth = self.latest_depth.copy() if self.latest_depth is not None else None
            if vis_color is not None:
                detection_utils.visualize_detections_masks(
                    part=2, detections=None,
                    rgb_image=vis_color, depth_image=vis_depth,
                )

            pose = self._detect()
            if pose is not None:
                print(f'[SEARCH] Found "{self.obj_name}" at pan={np.degrees(pan):.0f}°')
                self._reset_head()
                return pose

        print(f'[SEARCH] "{self.obj_name}" not found.')
        self._reset_head()
        return None

    def _reset_head(self):
        self.hello_node.move_to_pose(
            {'joint_head_pan': -1.6, 'joint_head_tilt': -0.5},
            blocking=True,
        )

    def destroy(self):
        self.destroy_node()
