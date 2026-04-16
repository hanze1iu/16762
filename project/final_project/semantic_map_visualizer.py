"""
semantic_map_visualizer.py
==========================
Publishes semantic_map.yaml contents as RViz markers.

  Zones   → green arrows  (nav_point direction = yaw)
  Dropoffs → red arrows

Usage:
    cd ~/16762/hanzel/16762/project/final_project
    python3 semantic_map_visualizer.py
Then in RViz: Add → MarkerArray → topic /semantic_map_markers
"""

import math
import yaml
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

SEMANTIC_MAP_PATH = 'semantic_map.yaml'


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SemanticMapVisualizer(Node):
    def __init__(self):
        super().__init__('semantic_map_visualizer')
        self.pub = self.create_publisher(MarkerArray, '/semantic_map_markers', 10)

        with open(SEMANTIC_MAP_PATH, 'r') as f:
            self.smap = yaml.safe_load(f) or {}

        # Publish at 1 Hz so RViz picks it up after connecting
        self.create_timer(1.0, self._publish)
        self.get_logger().info('SemanticMapVisualizer running — topic: /semantic_map_markers')

    def _make_arrow(self, mid, x, y, yaw, r, g, b, name):
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'semantic_map'
        arrow.id = mid
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = float(x)
        arrow.pose.position.y = float(y)
        arrow.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        arrow.pose.orientation.x = qx
        arrow.pose.orientation.y = qy
        arrow.pose.orientation.z = qz
        arrow.pose.orientation.w = qw
        arrow.scale.x = 0.4   # shaft length
        arrow.scale.y = 0.08  # shaft width
        arrow.scale.z = 0.08
        arrow.color.r = r
        arrow.color.g = g
        arrow.color.b = b
        arrow.color.a = 1.0
        return arrow

    def _make_text(self, mid, x, y, label, r, g, b):
        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'semantic_map'
        text.id = mid
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = 0.3
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2
        text.color.r = r
        text.color.g = g
        text.color.b = b
        text.color.a = 1.0
        text.text = label
        return text

    def _publish(self):
        array = MarkerArray()
        mid = 0

        # Zones — green
        for name, zone in (self.smap.get('zones') or {}).items():
            nav = zone.get('nav_point', {})
            x, y, yaw = nav.get('x', 0), nav.get('y', 0), nav.get('yaw', 0)
            array.markers.append(self._make_arrow(mid, x, y, yaw, 0.0, 0.8, 0.0, name))
            mid += 1
            array.markers.append(self._make_text(mid, x, y, f'zone: {name}', 0.0, 0.8, 0.0))
            mid += 1

        # Dropoffs — red
        for name, dropoff in (self.smap.get('dropoffs') or {}).items():
            x, y, yaw = dropoff.get('x', 0), dropoff.get('y', 0), dropoff.get('yaw', 0)
            array.markers.append(self._make_arrow(mid, x, y, yaw, 0.9, 0.1, 0.1, name))
            mid += 1
            array.markers.append(self._make_text(mid, x, y, f'dropoff: {name}', 0.9, 0.1, 0.1))
            mid += 1

        self.pub.publish(array)


def main():
    rclpy.init()
    node = SemanticMapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
