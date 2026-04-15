"""
fetch_demo.py
=============
Semantic Fetch — built directly on verified Lab 3 detection + grasp code.

Pipeline (sequential, single process):
  1. Load semantic_map.yaml  → object → zones → dropoff
  2. Navigate to zone        (navigation_utils.py / Nav2)
  3. Head sweep + detect     (lab3 object_detector_pcd logic, inlined)
  4. Grasp                   (lab3 grasp_objects logic, inlined)
  5. Stow → navigate to dropoff
  6. Place (lower arm + open gripper + stow)

Usage:
  python3 fetch_demo.py "water bottle"
  python3 fetch_demo.py "coffee cup" --dropoff table

Prerequisites:
  ros2 launch stretch_core stretch_driver.launch.py
  ros2 launch stretch_core d435i_low_resolution.launch.py
  ros2 launch stretch_nav2 navigation.launch.py map:=final_project/map/<map>.yaml
"""

import sys
import time
import threading
import argparse
import yaml
import os
import os.path as osp

import cv2
import numpy as np
import rclpy
import tf2_ros
import ikpy.utils.geometry
import message_filters

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped

import hello_helpers.hello_misc as hm
from navigation_utils import Navigator, NavResult
import detection_utils

# ik_ros_utils lives in lab3 (verified implementation)
sys.path.insert(0, osp.join(osp.dirname(__file__), '..', 'lab3'))
import ik_ros_utils as ik


# ---------------------------------------------------------------------------
# Constants  (mirroring lab3 defaults)
# ---------------------------------------------------------------------------

MODEL_PATH     = '/home/hello-robot/models/yoloe-26s-seg.pt'
COLOR_TOPIC    = '/camera/color/image_raw'
DEPTH_TOPIC    = '/camera/aligned_depth_to_color/image_raw'
CAM_INFO_TOPIC = '/camera/color/camera_info'

CONF_THRESHOLD = 0.25
MIN_DEPTH_MM   = 200    # same floor as lab3 (just > 0)
MAX_DEPTH_MM   = 3000

# Head sweep
HEAD_TILT      = -0.5
HEAD_PAN_STEPS = [-1.6, -0.8, 0.0, 0.8, 1.5]   # left → right
SETTLE_SEC     = 0.8    # wait after head move before capturing frame
FRAME_WAIT_SEC = 3.0    # timeout waiting for a fresh frame
DETECT_TRIES   = 2      # attempts per pan angle

# Grasp (mirrors lab3 IKTargetFollowing)
DELTA          = 0.05   # step size toward goal (m)
MAX_STEPS      = 40
GRASP_SETTLE   = 0.4    # seconds between IK steps

SEMANTIC_MAP_PATH   = 'semantic_map.yaml'
OBJECT_QUERIES_PATH = 'object_queries.yaml'


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def load_yaml(path: str) -> dict:
    with open(path) as f:
        data = yaml.safe_load(f)
    if not data:
        sys.exit(f'[ERROR] {path} is empty.')
    return data


def load_object_queries(path: str) -> dict:
    """Returns {object_name: [query1, query2, ...]} or {} if file missing."""
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def resolve_queries(obj_name: str, queries_map: dict) -> list:
    """Return YOLO query list for obj_name; fall back to [obj_name]."""
    return queries_map.get(obj_name, [obj_name])


def lookup_zones(obj_name: str, smap: dict) -> list:
    oz = smap.get('object_zones', {})
    if obj_name not in oz:
        sys.exit(f'[ERROR] "{obj_name}" not in semantic_map.yaml object_zones.\n'
                 f'        Known: {list(oz.keys())}')
    return oz[obj_name]


def get_zone(name: str, smap: dict) -> dict:
    z = smap.get('zones', {})
    if name not in z:
        sys.exit(f'[ERROR] Zone "{name}" not defined in semantic_map.yaml.')
    return z[name]


def get_dropoff(name: str, smap: dict) -> dict:
    d = smap.get('dropoffs', {})
    if name not in d:
        sys.exit(f'[ERROR] Drop-off "{name}" not in semantic_map.yaml.\n'
                 f'        Known: {list(d.keys())}')
    return d[name]


# ---------------------------------------------------------------------------
# FetchDemo — single HelloNode, lab3 detection + grasp inlined
# ---------------------------------------------------------------------------

class FetchDemo(hm.HelloNode):
    """
    Inherits HelloNode for move_to_pose / stow_the_robot / ROS2 spin thread.
    Detection and grasp logic are ported directly from lab3 verified code.
    """

    def __init__(self, obj_queries: list):
        hm.HelloNode.__init__(self)
        self.obj_queries = obj_queries   # YOLO prompts, e.g. ['water bottle', 'bottle']

        # Camera state
        self.bridge          = CvBridge()
        self.latest_color    = None
        self.latest_depth    = None
        self.latest_cam_info = None
        self._frame_lock     = threading.Lock()
        self._new_frame      = threading.Event()

        # Joint state
        self._joint_lock = threading.Lock()
        self.joint_state = {}

        # TF / YOLO — initialised in _setup() after the node is created
        self.tf_buffer  = None
        self.model      = None

    # ------------------------------------------------------------------
    # Setup (called inside main(), after HelloNode.main() creates the node)
    # ------------------------------------------------------------------

    def _setup(self):
        # Camera subscribers + synchroniser — identical to lab3
        self.color_sub    = message_filters.Subscriber(self, Image,      COLOR_TOPIC)
        self.depth_sub    = message_filters.Subscriber(self, Image,      DEPTH_TOPIC)
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, CAM_INFO_TOPIC)
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.cam_info_sub],
            queue_size=10, slop=0.05,
        )
        self.synchronizer.registerCallback(self._image_callback)

        # Joint state subscriber — identical to lab3
        self.create_subscription(JointState, '/stretch/joint_states',
                                 self._joint_callback, 1)

        # TF — identical to lab3
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # YOLO-E — identical to lab3, but with expanded query list
        from ultralytics import YOLO
        self.model = YOLO(MODEL_PATH)
        self.model.set_classes(self.obj_queries)
        print(f'[YOLO] Loaded model. Queries: {self.obj_queries}')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _image_callback(self, color_msg, depth_msg, cam_info_msg):
        """Identical to lab3 object_detector_pcd.image_callback."""
        try:
            color = cv2.rotate(
                self.bridge.imgmsg_to_cv2(color_msg, 'bgr8'),
                cv2.ROTATE_90_CLOCKWISE,
            )
            depth = cv2.rotate(
                self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough'),
                cv2.ROTATE_90_CLOCKWISE,
            )
        except CvBridgeError as e:
            print(f'[CAM] CvBridge error: {e}')
            return

        with self._frame_lock:
            self.latest_color    = color
            self.latest_depth    = depth
            self.latest_cam_info = cam_info_msg
            self._new_frame.set()

    def _joint_callback(self, msg: JointState):
        """Identical to lab3 grasp_objects.joint_states_callback."""
        names_needed = {
            'joint_lift', 'joint_arm_l0',
            'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll',
        }
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in names_needed:
                    self.joint_state[name] = pos

    # ------------------------------------------------------------------
    # Detection — lab3 get_goal_pose (Part 2 pointcloud centroid) inlined
    # ------------------------------------------------------------------

    def _detect_once(self) -> 'PoseStamped | None':
        """
        Run YOLO on the latest frame, compute pointcloud centroid over the
        mask (lab3 Part 2 method), return PoseStamped in camera frame.
        Returns None if no detection or insufficient depth points.
        """
        with self._frame_lock:
            if self.latest_color is None:
                return None
            color    = self.latest_color.copy()
            depth    = self.latest_depth.copy()
            cam_info = self.latest_cam_info

        results    = self.model(color, conf=CONF_THRESHOLD)
        detections = detection_utils.parse_results(results)

        # Visualise (main thread — safe here since we're on the calling thread)
        detection_utils.visualize_detections_masks(
            part=2, detections=detections,
            rgb_image=color, depth_image=depth,
        )

        if not detections:
            return None

        # Filter to our target queries (same as lab3 target_obj filter)
        targets = [d for d in detections if d['label'] in self.obj_queries]
        if not targets:
            targets = detections   # fall back to any detection
        target = targets[0]

        mask_polygon = target['mask']
        h_rot, w_rot = depth.shape[:2]
        mask_bin = np.zeros((h_rot, w_rot), dtype=np.uint8)
        cv2.fillPoly(mask_bin, [mask_polygon], 1)
        ys_rot, xs_rot = np.where(mask_bin)

        # Coordinate fix for 90° rotation — identical to lab3
        h_orig = cam_info.height
        points_3d = []
        for x_rot, y_rot in zip(xs_rot, ys_rot):
            z_mm = float(depth[y_rot, x_rot])
            if z_mm < MIN_DEPTH_MM or z_mm > MAX_DEPTH_MM or np.isnan(z_mm):
                continue
            x_orig = y_rot
            y_orig = h_orig - 1 - x_rot
            xyz = detection_utils.pixel_to_3d((x_orig, y_orig), z_mm, cam_info)
            if not np.any(np.isnan(xyz)):
                points_3d.append(xyz)

        if len(points_3d) < 10:
            print(f'[DETECT] Detection found but only {len(points_3d)} valid depth points — skipping.')
            return None

        centroid = np.mean(points_3d, axis=0)
        return detection_utils.get_pose_msg(
            cam_info.header.stamp,
            cam_info.header.frame_id,
            centroid,
        )

    def detect_with_head_sweep(self) -> 'PoseStamped | None':
        """
        Sweep head through HEAD_PAN_STEPS; at each angle try DETECT_TRIES times.
        Returns PoseStamped in base_link on success, None if nothing found.
        """
        print(f'[DETECT] Starting head sweep ({len(HEAD_PAN_STEPS)} angles) ...')

        for pan in HEAD_PAN_STEPS:
            self.move_to_pose(
                {'joint_head_pan': float(pan), 'joint_head_tilt': HEAD_TILT},
                blocking=True,
            )
            time.sleep(SETTLE_SEC)

            for attempt in range(DETECT_TRIES):
                self._new_frame.clear()
                if not self._new_frame.wait(timeout=FRAME_WAIT_SEC):
                    print(f'[DETECT]   pan={np.degrees(pan):.0f}° try {attempt+1}: no frame received')
                    continue

                pose_cam = self._detect_once()
                if pose_cam is not None:
                    try:
                        pose_base = self.tf_buffer.transform(pose_cam, 'base_link')
                        print(f'[DETECT] Found at pan={np.degrees(pan):.0f}°  '
                              f'base_link: x={pose_base.pose.position.x:.3f} '
                              f'y={pose_base.pose.position.y:.3f} '
                              f'z={pose_base.pose.position.z:.3f}')
                        return pose_base
                    except Exception as e:
                        print(f'[DETECT] TF transform failed: {e}')

        print('[DETECT] Not found after full sweep.')
        # Reset head to forward position
        self.move_to_pose({'joint_head_pan': -1.6, 'joint_head_tilt': -0.5}, blocking=True)
        return None

    # ------------------------------------------------------------------
    # Grasp — lab3 IKTargetFollowing.goal_callback inlined as a loop
    # ------------------------------------------------------------------

    def grasp(self, goal_pose: PoseStamped) -> bool:
        """
        Move to ready pose, then iteratively step toward goal_pose and close
        the gripper when within DELTA.  Mirrors lab3 goal_callback exactly.
        """
        goal_pos = np.array([
            goal_pose.pose.position.x,
            goal_pose.pose.position.y,
            goal_pose.pose.position.z,
        ])
        print(f'[GRASP] Target (base_link): '
              f'x={goal_pos[0]:.3f}  y={goal_pos[1]:.3f}  z={goal_pos[2]:.3f}')

        # --- ready pose (lab3 move_to_ready_pose) ---
        print('[GRASP] Moving to ready pose ...')
        self.move_to_pose({
            'joint_lift':        ik.READY_POSE_P2['joint_lift'],
            'joint_arm':         ik.READY_POSE_P2['joint_arm_l0'],
            'joint_wrist_yaw':   ik.READY_POSE_P2['joint_wrist_yaw'],
            'joint_wrist_pitch': ik.READY_POSE_P2['joint_wrist_pitch'],
            'gripper_aperture':  ik.READY_POSE_P2['gripper_aperture'],
        }, blocking=True)
        self.move_to_pose({
            'joint_head_pan':  ik.READY_POSE_P2['joint_head_pan'],
            'joint_head_tilt': ik.READY_POSE_P2['joint_head_tilt'],
        }, blocking=True)

        # --- wait for joint states ---
        deadline = time.time() + 5.0
        while time.time() < deadline:
            with self._joint_lock:
                if len(self.joint_state) >= 5:
                    break
            time.sleep(0.05)
        else:
            print('[GRASP] Timed out waiting for joint states.')
            return False

        # --- iterative approach (lab3 goal_callback loop) ---
        print(f'[GRASP] Iterative approach (max {MAX_STEPS} steps, delta={DELTA} m) ...')

        for step in range(MAX_STEPS):
            # Get gripper position (lab3 get_gripper_pose_in_base_frame)
            try:
                tf = self.tf_buffer.lookup_transform(
                    'base_link', 'link_grasp_center', rclpy.time.Time()
                )
                gripper_pos = ik.get_xyz_from_msg(tf)
            except Exception as e:
                print(f'[GRASP] TF error: {e}')
                return False

            diff = goal_pos - gripper_pos
            dist = np.linalg.norm(diff)
            print(f'[GRASP] step {step+1:02d}: dist={dist:.3f} m  '
                  f'gripper=({gripper_pos[0]:.3f},{gripper_pos[1]:.3f},{gripper_pos[2]:.3f})')

            if dist <= DELTA:
                # Close enough — grasp (lab3 line 103-105)
                print('[GRASP] Within reach — closing gripper.')
                self.move_to_pose({'gripper_aperture': -0.2}, blocking=True)
                self.move_to_pose({'joint_arm': 0.0}, blocking=True)
                print('[GRASP] Gripper closed.')
                return True

            # Compute waypoint (lab3 compute_waypoint_to_goal)
            direction   = diff / dist
            waypoint    = gripper_pos + direction * DELTA
            waypoint[2] = max(waypoint[2], goal_pos[2])   # never go below target
            orient      = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)

            # IK solve (lab3 lines 84-86)
            with self._joint_lock:
                js = dict(self.joint_state)
            q_init = ik.get_current_configuration(js)
            q_soln = ik.get_grasp_goal(waypoint, orient, q_init)

            if q_soln is None:
                print(f'[GRASP] IK: no solution at step {step+1}. Aborting.')
                return False

            ik.move_to_configuration(self, q_soln)
            time.sleep(GRASP_SETTLE)

        print(f'[GRASP] Reached max steps ({MAX_STEPS}) without grasping.')
        return False

    # ------------------------------------------------------------------
    # Placement  (M5)
    # ------------------------------------------------------------------

    def place(self):
        """Lower arm to ~table height, open gripper, stow."""
        print('[PLACE] Lowering arm ...')
        self.move_to_pose({'joint_lift': 0.5}, blocking=True)
        print('[PLACE] Opening gripper ...')
        self.move_to_pose({'gripper_aperture': 0.8}, blocking=True)
        time.sleep(0.5)
        self.stow_the_robot()
        print('[PLACE] Done.')

    # ------------------------------------------------------------------
    # Main pipeline
    # ------------------------------------------------------------------

    def run(self, obj_name: str, dropoff_name: str, smap: dict):
        print(f'\n{"="*54}')
        print(f'  Object   : "{obj_name}"')
        print(f'  Queries  : {self.obj_queries}')
        print(f'  Drop-off : "{dropoff_name}"')
        print(f'{"="*54}\n')

        zone_names = lookup_zones(obj_name, smap)
        dropoff    = get_dropoff(dropoff_name, smap)

        print(f'[CONFIG] Zones to search (priority order): {zone_names}')
        print(f'[CONFIG] Drop-off coords: x={dropoff["x"]}, y={dropoff["y"]}\n')

        self._setup()
        self.stow_the_robot()

        nav = Navigator(node=self)
        nav.wait_until_ready()

        # ----------------------------------------------------------------
        # Search each zone
        # ----------------------------------------------------------------
        goal_pose     = None
        found_in_zone = None

        for zone_name in zone_names:
            zone   = get_zone(zone_name, smap)
            nav_pt = zone['nav_point']

            print(f'[NAV] → zone "{zone_name}"  '
                  f'(x={nav_pt["x"]:.2f}, y={nav_pt["y"]:.2f}, yaw={nav_pt["yaw"]:.2f})')

            result = nav.go_to(nav_pt['x'], nav_pt['y'], nav_pt['yaw'])
            if result != NavResult.SUCCEEDED:
                print(f'[NAV] Could not reach "{zone_name}" ({result}), trying next zone.\n')
                continue

            print(f'[NAV] Arrived at "{zone_name}". Detecting ...\n')
            goal_pose = self.detect_with_head_sweep()

            if goal_pose is not None:
                found_in_zone = zone_name
                break

            print(f'[DETECT] "{obj_name}" not found in "{zone_name}".\n')

        if goal_pose is None:
            print(f'[FAIL] "{obj_name}" not found in any zone: {zone_names}')
            nav.shutdown()
            return False

        print(f'\n[FOUND] "{obj_name}" in zone "{found_in_zone}". Grasping ...\n')

        # ----------------------------------------------------------------
        # Grasp
        # ----------------------------------------------------------------
        if not self.grasp(goal_pose):
            print('[FAIL] Grasp failed.')
            nav.shutdown()
            return False

        # ----------------------------------------------------------------
        # Transport to drop-off
        # ----------------------------------------------------------------
        print('[NAV] Stowing arm and navigating to drop-off ...')
        self.stow_the_robot()

        result = nav.go_to(dropoff['x'], dropoff['y'], dropoff.get('yaw', 0.0))
        if result != NavResult.SUCCEEDED:
            print(f'[NAV] Could not reach drop-off "{dropoff_name}" ({result}).')
            nav.shutdown()
            return False

        # ----------------------------------------------------------------
        # Place
        # ----------------------------------------------------------------
        self.place()

        nav.shutdown()
        print(f'\n[SUCCESS] "{obj_name}" delivered to "{dropoff_name}".')
        return True

    def main(self, obj_name: str, dropoff_name: str, smap: dict):
        hm.HelloNode.main(
            self,
            node_name='fetch_demo',
            node_topic_namespace='fetch_demo',
            wait_for_first_pointcloud=False,
        )
        self.run(obj_name, dropoff_name, smap)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description='Semantic Fetch Demo — uses Lab 3 verified detection + grasp.'
    )
    p.add_argument('object',
                   help='Object name (must match an entry in semantic_map.yaml)')
    p.add_argument('--dropoff', default='default',
                   help='Drop-off location name (default: "default")')
    p.add_argument('--map',     default=SEMANTIC_MAP_PATH,
                   help=f'Path to semantic_map.yaml  (default: {SEMANTIC_MAP_PATH})')
    p.add_argument('--queries', default=OBJECT_QUERIES_PATH,
                   help=f'Path to object_queries.yaml (default: {OBJECT_QUERIES_PATH})')
    return p.parse_args()


def main():
    args        = parse_args()
    smap        = load_yaml(args.map)
    queries_map = load_object_queries(args.queries)
    obj_queries = resolve_queries(args.object, queries_map)

    node = FetchDemo(obj_queries)
    try:
        node.main(
            obj_name     = args.object,
            dropoff_name = args.dropoff,
            smap         = smap,
        )
    except KeyboardInterrupt:
        print('\n[INTERRUPTED] Fetch cancelled.')


if __name__ == '__main__':
    main()
