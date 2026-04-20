"""
fetch.py
========
CLI entry point for the Semantic Fetch pipeline.

Detection is handled by a separately-running object_detector_pcd node
(lab3 code) that publishes to /object_detector/goal_pose.
fetch.py handles navigation + grasping only.

Usage:
    # T4 — start detector first
    python3 ../lab3/object_detector_pcd-1.py

    # T5 — then run fetch
    python3 fetch.py "water bottle"
    python3 fetch.py "coffee mug" --dropoff table

Prerequisites:
    ros2 launch stretch_core stretch_driver.launch.py mode:=navigation
    ros2 launch stretch_core d435i_low_resolution.launch.py
    ros2 launch stretch_nav2 navigation.launch.py map:=final_project/map/<map>.yaml
"""

import sys
import time
import threading
import argparse
import yaml

import rclpy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import hello_helpers.hello_misc as hm
from navigation_utils import Navigator, NavResult
from grasp_pipeline import GraspPipeline

SEMANTIC_MAP_PATH   = 'semantic_map.yaml'
GOAL_TOPIC          = '/object_detector/goal_pose'
DETECTION_TIMEOUT   = 30.0   # seconds to wait for first detection after arriving

# Head pose for detection — camera points forward and slightly down toward desk level
READY_HEAD = {
    'joint_head_pan':  -1.6,
    'joint_head_tilt': -0.5,
}


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def load_semantic_map(path: str) -> dict:
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    if not data:
        sys.exit(f'[ERROR] {path} is empty.')
    return data


def lookup_zones(obj_name: str, smap: dict) -> tuple:
    """Returns (zone_list, grasp_delta)."""
    object_zones = smap.get('object_zones', {})
    if obj_name not in object_zones:
        known = list(object_zones.keys())
        sys.exit(
            f'[ERROR] Object "{obj_name}" not in semantic_map.yaml.\n'
            f'        Known objects: {known}\n'
            f'        Add it under object_zones in {SEMANTIC_MAP_PATH}.'
        )
    entry = object_zones[obj_name]
    zones = entry['zones']
    grasp_delta    = entry.get('grasp_delta', 0.03)
    grasp_z_offset = entry.get('grasp_z_offset', None)   # None = use pipeline default
    return zones, grasp_delta, grasp_z_offset


def get_zone(zone_name: str, smap: dict) -> dict:
    zones = smap.get('zones', {})
    if zone_name not in zones:
        sys.exit(f'[ERROR] Zone "{zone_name}" not defined in {SEMANTIC_MAP_PATH}.')
    return zones[zone_name]


def get_dropoff(dropoff_name: str, smap: dict) -> dict:
    dropoffs = smap.get('dropoffs', {})
    if dropoff_name not in dropoffs:
        known = list(dropoffs.keys())
        sys.exit(
            f'[ERROR] Drop-off "{dropoff_name}" not in semantic_map.yaml.\n'
            f'        Known drop-offs: {known}'
        )
    return dropoffs[dropoff_name]


PLACE_LIFT_HEIGHT = 0.76   # slightly above table (~80cm) so object clears edge

def place_object(node, dropoff: dict):
    print('[PLACE] Extending arm 16cm ...')
    node.move_to_pose({'joint_arm': 0.16}, blocking=True)

    print('[PLACE] Lowering to table height ...')
    node.move_to_pose({'joint_lift': PLACE_LIFT_HEIGHT}, blocking=True)

    print('[PLACE] Opening gripper ...')
    node.move_to_pose({'gripper_aperture': 0.6}, blocking=True)

    print('[PLACE] Object placed.')


# ---------------------------------------------------------------------------
# FetchNode
# ---------------------------------------------------------------------------

class FetchNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)

        # Latest goal pose from external detector (camera frame)
        self._goal_lock   = threading.Lock()
        self._latest_goal = None   # PoseStamped in camera frame

        # TF buffer for camera → base_link transform
        self._tf_buffer   = None
        self._tf_listener = None

    # ------------------------------------------------------------------
    # Setup (called after HelloNode.main() creates the node)
    # ------------------------------------------------------------------

    def _setup(self):
        self._tf_buffer    = tf2_ros.Buffer()
        self._tf_listener  = tf2_ros.TransformListener(self._tf_buffer, self)
        from rclpy.qos import QoSProfile, DurabilityPolicy
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._target_pub = self.create_publisher(String, '/fetch/target_object', qos)

        self.create_subscription(
            PoseStamped,
            GOAL_TOPIC,
            self._goal_cb,
            1,
        )
        print(f'[DETECT] Subscribed to {GOAL_TOPIC}')

    # ------------------------------------------------------------------
    # Goal callback — fired by HelloNode's spin thread
    # ------------------------------------------------------------------

    def _goal_cb(self, msg: PoseStamped):
        with self._goal_lock:
            self._latest_goal = msg

    # ------------------------------------------------------------------
    # Get latest goal in base_link
    # ------------------------------------------------------------------

    def get_goal_base_link(self) -> 'PoseStamped | None':
        """Transform the latest camera-frame goal to base_link."""
        with self._goal_lock:
            msg = self._latest_goal

        if msg is None:
            return None

        try:
            tf = self._tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
            )
            return do_transform_pose_stamped(msg, tf)
        except Exception as e:
            print(f'[DETECT] TF error: {e}')
            return None

    # ------------------------------------------------------------------
    # Wait for a valid detection after arriving at a zone
    # ------------------------------------------------------------------

    def _wait_for_detection(self, timeout: float = DETECTION_TIMEOUT) -> 'PoseStamped | None':
        print(f'[DETECT] Waiting up to {timeout:.0f}s for detection ...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            pose = self.get_goal_base_link()
            if pose is not None:
                print(f'[DETECT] Got pose: '
                      f'x={pose.pose.position.x:.3f}  '
                      f'y={pose.pose.position.y:.3f}  '
                      f'z={pose.pose.position.z:.3f}')
                return pose
            time.sleep(0.1)
        print('[DETECT] Timed out — no detection received.')
        return None

    # ------------------------------------------------------------------
    # Main pipeline
    # ------------------------------------------------------------------

    def run(self, obj_name: str, dropoff_name: str, smap: dict, stow: bool = True):
        print(f'\n{"="*52}')
        print(f'  Fetching : "{obj_name}"')
        print(f'  Drop-off : "{dropoff_name}"')
        print(f'{"="*52}\n')

        zone_names, grasp_delta, grasp_z_offset = lookup_zones(obj_name, smap)
        dropoff    = get_dropoff(dropoff_name, smap)

        print(f'[CONFIG] Zones: {zone_names}')
        print(f'[CONFIG] Drop-off: x={dropoff["x"]}, y={dropoff["y"]}\n')

        self._setup()
        if stow:
            print('[INIT] Retracting arm before stow ...')
            self.move_to_pose({'joint_arm': 0.0}, blocking=True)
            self.stow_the_robot()
        else:
            # In-task mode: only retract arm, keep lift/wrist/gripper as-is
            print('[INIT] In-task mode — retracting arm only ...')
            self.move_to_pose({'joint_arm': 0.0}, blocking=True)

        # Tell detector which object to look for
        msg = String()
        msg.data = obj_name
        self._target_pub.publish(msg)
        print(f'[DETECT] Published target → "{obj_name}"')

        nav = Navigator(node=self)
        nav.wait_until_ready()

        grasper = GraspPipeline(self)

        # ---- search each zone ----
        goal_pose     = None
        found_in_zone = None

        for zone_name in zone_names:
            zone   = get_zone(zone_name, smap)
            nav_pt = zone['nav_point']

            print(f'[NAV] → zone "{zone_name}"  '
                  f'(x={nav_pt["x"]:.2f}, y={nav_pt["y"]:.2f}, yaw={nav_pt["yaw"]:.2f})')

            result = nav.go_to(nav_pt['x'], nav_pt['y'], nav_pt['yaw'])
            if result != NavResult.SUCCEEDED:
                print(f'[NAV] Could not reach "{zone_name}" ({result}), skipping.\n')
                continue

            print(f'[NAV] Arrived at "{zone_name}". Setting ready head pose ...')
            self.move_to_pose(READY_HEAD, blocking=True)

            # Clear stale detection before waiting for a fresh one
            with self._goal_lock:
                self._latest_goal = None

            print('[DETECT] Waiting for detection ...\n')
            goal_pose = self._wait_for_detection()
            if goal_pose is not None:
                found_in_zone = zone_name
                break

            print(f'[DETECT] "{obj_name}" not detected in "{zone_name}".\n')

        if goal_pose is None:
            print(f'[FAIL] "{obj_name}" not found in any zone.')
            nav.shutdown()
            return False

        print(f'\n[FOUND] Detected in zone "{found_in_zone}". Grasping ...\n')

        # ---- M4: grasp (pass live detection function for continuous update) ----
        print(f'[GRASP] Using grasp_delta={grasp_delta}, z_offset={grasp_z_offset} for "{obj_name}"')
        success = grasper.grasp(goal_pose, get_goal_fn=self.get_goal_base_link,
                                delta=grasp_delta, z_offset=grasp_z_offset)
        if not success:
            print('[FAIL] Grasp failed.')
            nav.shutdown()
            return False

        print('[GRASP] Object secured. Navigating to drop-off ...\n')

        result = nav.go_to(dropoff['x'], dropoff['y'], dropoff.get('yaw', 0.0))
        if result != NavResult.SUCCEEDED:
            print(f'[NAV] Could not reach drop-off ({result}).')
            nav.shutdown()
            return False

        place_object(self, dropoff)
        nav.shutdown()
        print(f'\n[SUCCESS] "{obj_name}" delivered to "{dropoff_name}".')
        return True

    def main(self, tasks: list, smap: dict, intask: bool = False):
        hm.HelloNode.main(
            self,
            node_name='fetch_node',
            node_topic_namespace='fetch_node',
            wait_for_first_pointcloud=False,
        )
        for i, (obj_name, dropoff_name) in enumerate(tasks):
            print(f'\n[TASK {i+1}/{len(tasks)}] "{obj_name}" → "{dropoff_name}"')
            stow = (i == 0) and not intask
            success = self.run(obj_name, dropoff_name, smap, stow=stow)
            if not success:
                print(f'[TASK {i+1}] Failed — stopping task queue.')
                break


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description='Semantic Fetch — navigation + grasp. '
                    'Run object_detector_pcd.py separately for detection.'
    )
    parser.add_argument('object', nargs='?', default=None,
                        help='Object name (single-task mode)')
    parser.add_argument('--dropoff', default='default',
                        help='Drop-off location name (single-task mode, default: "default")')
    parser.add_argument('--task', metavar=('OBJECT', 'DROPOFF'), nargs=2,
                        action='append', dest='tasks',
                        help='Multi-task mode: --task "water bottle" desk2 --task "cup" desk3')
    parser.add_argument('--intask', action='store_true',
                        help='In-task mode: skip stow, retract arm only')
    parser.add_argument('--map', default=SEMANTIC_MAP_PATH,
                        help=f'Path to semantic_map.yaml (default: {SEMANTIC_MAP_PATH})')
    return parser.parse_args()


def main():
    args = parse_args()
    smap = load_semantic_map(args.map)

    # Build task list
    if args.tasks:
        tasks = [(obj, dropoff) for obj, dropoff in args.tasks]
    elif args.object:
        tasks = [(args.object, args.dropoff)]
    else:
        sys.exit('[ERROR] Specify an object or use --task OBJECT DROPOFF.')

    node = FetchNode()
    try:
        node.main(tasks=tasks, smap=smap, intask=args.intask)
    except KeyboardInterrupt:
        print('\n[INTERRUPTED] Fetch cancelled.')


if __name__ == '__main__':
    main()
