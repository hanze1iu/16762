"""
fetch.py
========
CLI entry point for the Semantic Fetch pipeline.

Usage:
    python3 fetch.py <object_name> [--dropoff <name>] [--map <path>]

Examples:
    python3 fetch.py "water bottle"
    python3 fetch.py "coffee mug" --dropoff table

Pipeline:
    M1  semantic_map.yaml  ← object → zone lookup
    M2  navigation_utils   ← navigate to zone nav_point         [DONE]
    M3  search_behavior    ← head sweep + orbit search          [DONE]
    M4  grasp_pipeline     ← detect → 3D pose → IK grasp       [stub]
    M5  (inline)           ← transport + placement              [stub]

Prerequisites:
    ros2 launch stretch_core stretch_driver.launch.py
    ros2 launch stretch_core d435i_low_resolution.launch.py
    ros2 launch stretch_nav2 navigation.launch.py map:=final_project/map/<map>.yaml
"""

import sys
import argparse
import yaml

import hello_helpers.hello_misc as hm
from navigation_utils import Navigator, NavResult
from search_behavior import SearchBehavior

SEMANTIC_MAP_PATH = 'semantic_map.yaml'


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def load_semantic_map(path: str) -> dict:
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    if not data:
        sys.exit(f'[ERROR] {path} is empty.')
    return data


def lookup_zones(obj_name: str, smap: dict) -> list:
    object_zones = smap.get('object_zones', {})
    if obj_name not in object_zones:
        known = list(object_zones.keys())
        sys.exit(
            f'[ERROR] Object "{obj_name}" not in semantic_map.yaml.\n'
            f'        Known objects: {known}\n'
            f'        Add it under object_zones in {SEMANTIC_MAP_PATH}.'
        )
    return object_zones[obj_name]


def get_zone(zone_name: str, smap: dict) -> dict:
    zones = smap.get('zones', {})
    if zone_name not in zones:
        sys.exit(
            f'[ERROR] Zone "{zone_name}" not defined under zones '
            f'in {SEMANTIC_MAP_PATH}.'
        )
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


# ---------------------------------------------------------------------------
# M4 stub — replaced when grasp_pipeline.py is written
# ---------------------------------------------------------------------------

def grasp_object(node, goal_pose):
    """
    [M4 stub] Approach and grasp the detected object.
    goal_pose: PoseStamped in base_link frame returned by SearchBehavior.
    Returns True on success.
    """
    # TODO: replace with:
    #   from grasp_pipeline import GraspPipeline
    #   grasper = GraspPipeline(node)
    #   return grasper.grasp(goal_pose)
    print('[GRASP] M4 stub — not implemented yet.')
    return False


# ---------------------------------------------------------------------------
# M5 stub — replaced when placement logic is written
# ---------------------------------------------------------------------------

def place_object(node, dropoff: dict):
    """
    [M5 stub] Lower arm to table height, open gripper, stow arm.
    """
    # TODO: implement:
    #   node.move_to_pose({'joint_lift': <table_height>}, blocking=True)
    #   node.move_to_pose({'gripper_aperture': 0.6}, blocking=True)
    #   node.stow_the_robot()
    print('[PLACE] M5 stub — not implemented yet.')


# ---------------------------------------------------------------------------
# Main pipeline (runs inside HelloNode context)
# ---------------------------------------------------------------------------

class FetchNode(hm.HelloNode):
    """
    HelloNode subclass that orchestrates the full fetch pipeline.
    Inheriting from HelloNode gives us:
      - move_to_pose() for head/arm/base control
      - stow_the_robot()
      - Properly initialised ROS2 node + background spin thread
    """

    def __init__(self):
        hm.HelloNode.__init__(self)

    def run(self, obj_name: str, dropoff_name: str, smap: dict):
        print(f'\n{"="*52}')
        print(f'  Fetching : "{obj_name}"')
        print(f'  Drop-off : "{dropoff_name}"')
        print(f'{"="*52}\n')

        # ---- resolve config ----
        zone_names = lookup_zones(obj_name, smap)
        dropoff    = get_dropoff(dropoff_name, smap)

        print(f'[CONFIG] Zones to search (in order): {zone_names}')
        print(f'[CONFIG] Drop-off coords: x={dropoff["x"]}, y={dropoff["y"]}\n')

        # ---- stow arm before driving ----
        self.stow_the_robot()

        # ---- start Nav2 ----
        nav = Navigator()
        nav.wait_until_ready()

        # ---- build searcher once (loads YOLO-E, sets up camera subs) ----
        searcher = SearchBehavior(self, nav, obj_name)
        print(f'[SEARCH] YOLO-E loaded. Searching for "{obj_name}".\n')

        # ---- search each zone in priority order ----
        goal_pose     = None
        found_in_zone = None

        for zone_name in zone_names:
            zone   = get_zone(zone_name, smap)
            nav_pt = zone['nav_point']

            print(f'[NAV] → zone "{zone_name}"  '
                  f'(x={nav_pt["x"]:.2f}, y={nav_pt["y"]:.2f}, '
                  f'yaw={nav_pt["yaw"]:.2f})')

            result = nav.go_to(nav_pt['x'], nav_pt['y'], nav_pt['yaw'])

            if result != NavResult.SUCCEEDED:
                print(f'[NAV] Could not reach "{zone_name}" ({result}), trying next zone.\n')
                continue

            print(f'[NAV] Arrived at "{zone_name}". Starting search ...\n')
            goal_pose = searcher.search(zone)

            if goal_pose is not None:
                found_in_zone = zone_name
                break

            print(f'[SEARCH] "{obj_name}" not found in "{zone_name}".\n')

        # ---- object not found anywhere ----
        if goal_pose is None:
            print(f'[FAIL] "{obj_name}" was not found in zones: {zone_names}')
            nav.shutdown()
            return False

        print(f'\n[FOUND] "{obj_name}" located in zone "{found_in_zone}". Grasping ...\n')

        # ---- M4: grasp ----
        success = grasp_object(self, goal_pose)
        if not success:
            print('[FAIL] Grasp failed.')
            nav.shutdown()
            return False

        print('[GRASP] Object secured. Navigating to drop-off ...\n')

        # ---- stow arm for safe transport ----
        self.stow_the_robot()

        # ---- M5a: navigate to drop-off ----
        result = nav.go_to(dropoff['x'], dropoff['y'], dropoff.get('yaw', 0.0))
        if result != NavResult.SUCCEEDED:
            print(f'[NAV] Could not reach drop-off "{dropoff_name}" ({result}).')
            nav.shutdown()
            return False

        # ---- M5b: place object ----
        place_object(self, dropoff)

        nav.shutdown()
        print(f'\n[SUCCESS] "{obj_name}" delivered to "{dropoff_name}".')
        return True

    def main(self, obj_name: str, dropoff_name: str, smap: dict):
        # HelloNode.main() initialises rclpy, creates the node, and starts a
        # background spin thread — then returns so we can run sequential code.
        hm.HelloNode.main(
            self,
            node_name='fetch_node',
            node_topic_namespace='fetch_node',
            wait_for_first_pointcloud=False,
        )
        self.run(obj_name, dropoff_name, smap)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description='Semantic Fetch — tell Stretch to fetch an object by name.'
    )
    parser.add_argument(
        'object',
        help='Object to fetch (must match an entry in semantic_map.yaml). '
             'Use quotes for multi-word names: "water bottle".'
    )
    parser.add_argument(
        '--dropoff', default='default',
        help='Drop-off location name (defined in semantic_map.yaml). Default: "default".'
    )
    parser.add_argument(
        '--map', default=SEMANTIC_MAP_PATH,
        help=f'Path to semantic_map.yaml. Default: {SEMANTIC_MAP_PATH}'
    )
    return parser.parse_args()


def main():
    args  = parse_args()
    smap  = load_semantic_map(args.map)
    node  = FetchNode()
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
