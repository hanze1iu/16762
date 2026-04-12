"""
fetch.py
========
CLI entry point for the Semantic Fetch pipeline.

Usage:
    python3 fetch.py <object_name> [--dropoff <name>]

Examples:
    python3 fetch.py "water bottle"
    python3 fetch.py "coffee mug" --dropoff table

Pipeline (each step is a separate module):
    1. [M2] Look up object → zone in semantic_map.yaml
    2. [M2] Navigate to zone nav_point
    3. [M3] Search the zone (head sweep + orbit)
    4. [M4] Grasp the detected object
    5. [M5] Navigate to drop-off location
    6. [M5] Place the object

Prerequisites:
    ros2 launch stretch_core stretch_driver.launch.py
    ros2 launch stretch_core d435i_low_resolution.launch.py
    ros2 launch stretch_nav2 navigation.launch.py map:=final_project/map/<map>.yaml
"""

import sys
import argparse
import yaml
import rclpy

from navigation_utils import Navigator, NavResult

SEMANTIC_MAP_PATH = 'semantic_map.yaml'


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def load_semantic_map(path: str) -> dict:
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    if not data:
        sys.exit(f'[ERROR] {path} is empty.')
    return data


def lookup_zones(obj_name: str, smap: dict) -> list:
    """
    Return the ordered list of zone names for a given object.
    Exits with an error message if the object is not in the map.
    """
    object_zones = smap.get('object_zones', {})
    if obj_name not in object_zones:
        known = list(object_zones.keys())
        sys.exit(
            f'[ERROR] Object "{obj_name}" not found in semantic_map.yaml.\n'
            f'        Known objects: {known}\n'
            f'        Add it under object_zones in {SEMANTIC_MAP_PATH}.'
        )
    return object_zones[obj_name]   # list of zone name strings


def get_zone(zone_name: str, smap: dict) -> dict:
    """Return zone config dict, exit if missing."""
    zones = smap.get('zones', {})
    if zone_name not in zones:
        sys.exit(
            f'[ERROR] Zone "{zone_name}" referenced in object_zones '
            f'but not defined under zones in {SEMANTIC_MAP_PATH}.'
        )
    return zones[zone_name]


def get_dropoff(dropoff_name: str, smap: dict) -> dict:
    """Return drop-off coord dict, exit if missing."""
    dropoffs = smap.get('dropoffs', {})
    if dropoff_name not in dropoffs:
        known = list(dropoffs.keys())
        sys.exit(
            f'[ERROR] Drop-off "{dropoff_name}" not found in semantic_map.yaml.\n'
            f'        Known drop-offs: {known}'
        )
    return dropoffs[dropoff_name]


# ---------------------------------------------------------------------------
# Pipeline stubs (replaced by M3 / M4 / M5 modules)
# ---------------------------------------------------------------------------

def search_zone(zone: dict, obj_name: str):
    """
    [M3 stub] Search the zone for the target object.
    Returns a 3-D goal pose (PoseStamped in base_link frame) or None.
    """
    # TODO: replace with search_behavior.search(zone, obj_name)
    print(f'[SEARCH] Stub — M3 not implemented yet. Pretending object was found.')
    return None          # None signals "not found"


def grasp_object(goal_pose):
    """
    [M4 stub] Approach and grasp the detected object.
    Returns True on success.
    """
    # TODO: replace with grasp_pipeline.grasp(goal_pose)
    print('[GRASP] Stub — M4 not implemented yet.')
    return False


def place_object(nav: Navigator, dropoff: dict):
    """
    [M5 stub] Navigate to drop-off and open gripper.
    """
    # TODO: replace with full M5 placement logic
    print('[PLACE] Stub — M5 not implemented yet.')


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def run_fetch(obj_name: str, dropoff_name: str, smap: dict):
    print(f'\n{"="*50}')
    print(f'  Fetching: "{obj_name}"  →  drop-off: "{dropoff_name}"')
    print(f'{"="*50}\n')

    # ---- resolve config ----
    zone_names = lookup_zones(obj_name, smap)
    dropoff    = get_dropoff(dropoff_name, smap)

    print(f'[CONFIG] Will search zones in order: {zone_names}')
    print(f'[CONFIG] Drop-off "{dropoff_name}": x={dropoff["x"]}, y={dropoff["y"]}\n')

    # ---- start Nav2 ----
    nav = Navigator()
    nav.wait_until_ready()

    # ---- search each zone in priority order ----
    goal_pose = None
    found_in_zone = None

    for zone_name in zone_names:
        zone = get_zone(zone_name, smap)
        nav_pt = zone['nav_point']

        print(f'[NAV] Navigating to zone "{zone_name}" '
              f'(x={nav_pt["x"]}, y={nav_pt["y"]}, yaw={nav_pt["yaw"]:.2f})')

        result = nav.go_to(nav_pt['x'], nav_pt['y'], nav_pt['yaw'])

        if result != NavResult.SUCCEEDED:
            print(f'[NAV] Could not reach zone "{zone_name}" ({result}), trying next zone.')
            continue

        print(f'[NAV] Arrived at zone "{zone_name}". Starting search...\n')

        # ---- M3: search ----
        goal_pose = search_zone(zone, obj_name)

        if goal_pose is not None:
            found_in_zone = zone_name
            break

        print(f'[SEARCH] "{obj_name}" not found in zone "{zone_name}".')

    # ---- object not found anywhere ----
    if goal_pose is None:
        print(f'\n[FAIL] "{obj_name}" was not found in any of the searched zones: {zone_names}')
        nav.shutdown()
        return False

    print(f'\n[FOUND] "{obj_name}" detected in zone "{found_in_zone}". Proceeding to grasp.\n')

    # ---- M4: grasp ----
    success = grasp_object(goal_pose)
    if not success:
        print(f'[FAIL] Grasp failed.')
        nav.shutdown()
        return False

    print('[GRASP] Object secured. Navigating to drop-off.\n')

    # ---- M5: transport + place ----
    result = nav.go_to(dropoff['x'], dropoff['y'], dropoff.get('yaw', 0.0))
    if result != NavResult.SUCCEEDED:
        print(f'[NAV] Could not reach drop-off "{dropoff_name}" ({result}).')
        nav.shutdown()
        return False

    place_object(nav, dropoff)

    print(f'\n[SUCCESS] "{obj_name}" delivered to "{dropoff_name}".')
    nav.shutdown()
    return True


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description='Semantic Fetch: tell the robot to fetch an object by name.'
    )
    parser.add_argument(
        'object',
        help='Name of the object to fetch (must match an entry in semantic_map.yaml). '
             'Use quotes for multi-word names, e.g. "water bottle".'
    )
    parser.add_argument(
        '--dropoff',
        default='default',
        help='Drop-off location name (defined in semantic_map.yaml). '
             'Default: "default".'
    )
    parser.add_argument(
        '--map',
        default=SEMANTIC_MAP_PATH,
        help=f'Path to semantic_map.yaml. Default: {SEMANTIC_MAP_PATH}'
    )
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()

    smap = load_semantic_map(args.map)

    try:
        run_fetch(
            obj_name     = args.object,
            dropoff_name = args.dropoff,
            smap         = smap,
        )
    except KeyboardInterrupt:
        print('\n[INTERRUPTED] Fetch cancelled by user.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
