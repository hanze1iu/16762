#! /usr/bin/env python3

# Adapted from the simple commander demo examples on
# https://github.com/ros-planning/navigation2/blob/galactic/nav2_simple_commander/nav2_simple_commander/demo_security.py

import math
from copy import deepcopy

import cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from stretch_nav2.robot_navigator import BasicNavigator, TaskResult


"""
Autonomous navigation to 4 distinct base poses (position + orientation) in the
AI Maker Space, while recording a video from the robot's head-mounted camera.
"""


def yaw_to_quaternion(yaw):
    """Convert a yaw angle (radians) to the (z, w) quaternion components."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class HeadCameraRecorder(Node):
    """Subscribes to the head camera color stream and writes frames to a video file."""

    def __init__(self, output_path='navigation_video.avi', fps=10):
        super().__init__('head_camera_recorder')
        self.bridge = CvBridge()
        self.video_writer = None
        self.output_path = output_path
        self.fps = fps

        # Head camera color topic — same topic used in object_detector_pcd.py
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info(f'HeadCameraRecorder ready; will save to "{output_path}"')

    def image_callback(self, msg):
        try:
            # The head camera on Stretch is mounted sideways; rotate for upright video
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

            # Lazily create the VideoWriter once we know the actual frame size
            if self.video_writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
                self.get_logger().info(f'VideoWriter opened: {w}x{h} @ {self.fps} fps')

            self.video_writer.write(frame)
        except CvBridgeError as e:
            self.get_logger().warn(f'CvBridge error: {e}')

    def stop(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            self.get_logger().info(f'Video saved to "{self.output_path}"')


def make_pose(navigator, x, y, z,w):
    """Build a PoseStamped in the map frame from (x, y, yaw_rad)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    # qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


def main():
    rclpy.init()

    navigator = BasicNavigator()
    recorder = HeadCameraRecorder(output_path='navigation_video.avi', fps=10)
    try:

    # -----------------------------------------------------------------------
    # 4 target poses in the AI Maker Space: [x (m), y (m), yaw (rad)]
    # • None of these is the start pose (0, 0, 0).
    # • Every pair is at least 2 m apart.
    # • Adjust these coordinates to match your actual map after localizing.
    # -----------------------------------------------------------------------
        waypoint_defs = [
            [0.0,  0.0,  0.0, 1.0  ],   # Pose 0 – starting pose
            [1.4, -1.0, -0.6, 0.8  ],   # Pose 1 – facing +x
            [2.3, -4.6, -0.0, 1.0  ],   # Pose 2 – facing +y  (~3 m from Pose 1)
            [0.7, -0.05, 0.0, 1.0  ],   # Pose 3 – facing -x  (~3 m from Pose 2)
            [2.9, 0.27, 0.14, 0.97  ],   # Pose 4 – facing -y  (~2.5 m from Pose 3)
        ]

        # ------ Set the initial pose (starting position in the map) ------
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        navigator.setInitialPose(initial_pose)

        # Wait for Nav2 to fully activate
        navigator.waitUntilNav2Active()

        # Build the list of goal PoseStampeds from the waypoint definitions
        route_poses = [make_pose(navigator, x, y, z, w) for x, y, z, w in waypoint_defs]

        # -----------------------------------------------------------------------
        # OPTION A (default): send all waypoints at once with followWaypoints
        # -----------------------------------------------------------------------
        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(route_poses)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            # Drive the recorder's subscription callback so frames keep being written
            rclpy.spin_once(recorder, timeout_sec=0.05)

            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                navigator.get_logger().info(
                    f'Waypoint {feedback.current_waypoint + 1}/{len(route_poses)}'
                )
                now = navigator.get_clock().now()
                if now - nav_start > Duration(seconds=600.0):
                    navigator.cancelTask()

        # -----------------------------------------------------------------------
        # OPTION B (alternative): visit each pose individually with goToPose.
        # Uncomment this block and comment out OPTION A above to use it.
        # -----------------------------------------------------------------------
        # nav_start = navigator.get_clock().now()
        # for idx, pose in enumerate(route_poses):
        #     navigator.get_logger().info(f'Navigating to pose {idx + 1}/{len(route_poses)}')
        #     navigator.goToPose(pose)
        #     while not navigator.isTaskComplete():
        #         rclpy.spin_once(recorder, timeout_sec=0.05)
        #         now = navigator.get_clock().now()
        #         if now - nav_start > Duration(seconds=600.0):
        #             navigator.cancelTask()
        #             break
        # -----------------------------------------------------------------------

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('All waypoints reached successfully!')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().info('Navigation was canceled.')
        elif result == TaskResult.FAILED:
            navigator.get_logger().info('Navigation failed.')

    except KeyboardInterrupt:
        navigator.get_logger().info('Interrupted by user, cancelling...')
        navigator.cancelTask()
    finally:
        recorder.stop()
        recorder.destroy_node()
        navigator.destroy_node()
        rclpy.shutdown()   


if __name__ == '__main__':
    main()
