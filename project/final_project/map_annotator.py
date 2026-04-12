"""
map_annotator.py
================
Interactive ROS2 tool for recording semantic zone and drop-off coordinates
into semantic_map.yaml.

WORKFLOW
--------
1. Build SLAM map and launch Nav2 with the saved map.
2. Open RViz (stretch_simple_test.rviz or nav2_default_view).
3. Run this node:
       ros2 run ... OR  python3 map_annotator.py
4. In RViz, click the "Publish Point" button (PointStamped icon) then click
   anywhere on the map. This node receives that click and prompts you in the
   terminal to give it a label.
5. When done, press Ctrl-C. The node writes all recorded points to
   semantic_map.yaml (merges with any existing entries).

LABEL CONVENTION (follow exactly, the node parses these):
  zone <name>          → zone nav_point  (e.g.  "zone kitchen")
  orbit <name> <n>     → zone orbit waypoint  (e.g.  "orbit kitchen 1")
  dropoff <name>       → drop-off location  (e.g.  "dropoff table")

Robot current pose (yaw) is read from the TF tree (map → base_link),
so make sure the robot is localised before recording zone centres.

Dependencies (already on Stretch):
  rclpy, tf2_ros, geometry_msgs, yaml
"""

import sys
import math
import yaml
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros

SEMANTIC_MAP_PATH = 'semantic_map.yaml'


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_yaml(path):
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        data = {}
    # Ensure top-level keys exist
    data.setdefault('dropoffs', {})
    data.setdefault('zones', {})
    data.setdefault('object_zones', {})
    return data


def save_yaml(path, data):
    with open(path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f'  [saved] {path}')


def get_robot_yaw(tf_buffer):
    """Return current robot yaw in the map frame, or 0.0 if unavailable."""
    try:
        t = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        q = t.transform.rotation
        # quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    except Exception:
        return 0.0


def parse_label(label_str):
    """
    Parse a user label string into a (kind, name, extra) tuple.
      "zone kitchen"      → ('zone',   'kitchen', None)
      "orbit kitchen 1"   → ('orbit',  'kitchen', '1')
      "dropoff table"     → ('dropoff','table',   None)
    Returns None on bad input.
    """
    parts = label_str.strip().split()
    if len(parts) < 2:
        return None
    kind = parts[0].lower()
    if kind == 'zone' and len(parts) == 2:
        return ('zone', parts[1], None)
    if kind == 'orbit' and len(parts) == 3:
        return ('orbit', parts[1], parts[2])
    if kind == 'dropoff' and len(parts) == 2:
        return ('dropoff', parts[1], None)
    return None


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class MapAnnotator(Node):
    def __init__(self):
        super().__init__('map_annotator')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.data = load_yaml(SEMANTIC_MAP_PATH)
        self._pending_point = None
        self._lock = threading.Lock()

        self.sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self._clicked_point_cb,
            10
        )

        self.get_logger().info(
            '\n'
            '======================================\n'
            '  Map Annotator ready.\n'
            '  In RViz: click "Publish Point" then\n'
            '  click on the map to record a point.\n'
            '  Label format:\n'
            '    zone <name>          (nav centre)\n'
            '    orbit <name> <n>     (orbit waypoint)\n'
            '    dropoff <name>       (drop-off spot)\n'
            '  Ctrl-C to finish and save.\n'
            '======================================'
        )

    # ------------------------------------------------------------------
    def _clicked_point_cb(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        yaw = get_robot_yaw(self.tf_buffer)

        print(f'\n[CLICK] x={x:.3f}  y={y:.3f}  robot_yaw={math.degrees(yaw):.1f}°')
        print('  Enter label (or ENTER to skip): ', end='', flush=True)

        # Read label in a background thread so we don't block the ROS spin
        def _prompt():
            label_str = sys.stdin.readline().strip()
            if not label_str:
                print('  [skipped]')
                return
            parsed = parse_label(label_str)
            if parsed is None:
                print(f'  [ERROR] Bad label format: "{label_str}"')
                print('  Expected:  zone <name> | orbit <name> <n> | dropoff <name>')
                return
            self._record(x, y, yaw, *parsed)

        t = threading.Thread(target=_prompt, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    def _record(self, x, y, yaw, kind, name, extra):
        coord = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)}

        if kind == 'dropoff':
            self.data['dropoffs'][name] = {**coord, 'description': ''}
            print(f'  [recorded] dropoff "{name}" → {coord}')

        elif kind == 'zone':
            if name not in self.data['zones']:
                self.data['zones'][name] = {
                    'description': '',
                    'nav_point': coord,
                    'orbit_waypoints': []
                }
            else:
                self.data['zones'][name]['nav_point'] = coord
            print(f'  [recorded] zone "{name}" nav_point → {coord}')

        elif kind == 'orbit':
            if name not in self.data['zones']:
                self.data['zones'][name] = {
                    'description': '',
                    'nav_point': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
                    'orbit_waypoints': []
                }
            self.data['zones'][name]['orbit_waypoints'].append(coord)
            n = len(self.data['zones'][name]['orbit_waypoints'])
            print(f'  [recorded] zone "{name}" orbit waypoint #{n} → {coord}')

        # Auto-save after every point so Ctrl-C never loses data
        save_yaml(SEMANTIC_MAP_PATH, self.data)

    # ------------------------------------------------------------------
    def shutdown(self):
        save_yaml(SEMANTIC_MAP_PATH, self.data)
        print('\n[done] Final semantic_map.yaml written.')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = MapAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
