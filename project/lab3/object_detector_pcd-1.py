import cv2
import yaml
import rclpy
import os.path as osp
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import detection_utils
import message_filters
import numpy as np


# Don't forget to start the camera before starting this node!
# Part 1: using in-gripper camera
#    ros2 launch stretch_core d405_basic.launch.py
# Part 2: using head camera
#    ros2 launch stretch_core d435i_low_resolution.launch.py
#
# ros2 run rviz2 rviz2 -d `ros2 pkg prefix --share stretch_calibration`/rviz/stretch_simple_test.rviz


class YOLOEObjectDetector(Node):
    def __init__(self, obj_queries, target_obj=None):
        super().__init__('yoloe_object_detector')
        self.visualize = True
        # target object to grasp; if None, defaults to the first detection
        self.target_obj = target_obj

        # ----------- Camera Streaming Setup -----------

        # subscribe to the robot's color and aligned depth camera image topics from the gripper camera
        # using message_filters, instead of self.create_subscription() to allow us
        #   to synchronize the two camera streams can use a single callback that triggers when both come in
        # TODO: ------------- start --------------
        # leave as is for part 1, 
        # change for part 2 to use the head camera
        self.color_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        self.color_cam_info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/color/camera_info')
        # TODO: -------------- end ---------------
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_cam_info = None

        # Use ApproximateTimeSynchronizer and register a callback function that runs within some time tolerance of when both images are received
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.color_cam_info_sub],
            queue_size=10,
            slop=0.01  # 10ms tolerance
        )
        self.synchronizer.registerCallback(self.image_callback)

        # bridge to convert ROS2 image messages to OpenCV images
        self.bridge = CvBridge()

        # -----------------------------------------------------

        # ----------- YOLO-E Object Detection SetuP -----------

        # Load the YOLOE model, which should already saved to common models directory on the robot
        #   we use yolo-e-v26-small for its high performance and low latency on limited compute
        model_path = '/home/hello-robot/models'
        model_name = 'yoloe-26s-seg.pt'
        self.model = YOLO(osp.join(model_path, model_name))

        # pass prompt for the object/s you want to detect
        self.obj_queries = obj_queries
        self.model.set_classes(self.obj_queries)

        # Run the detector and goals at a fixed frequency to reduce latency introduced by the detector
        #   and give the robot time to move between poses
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_goals_callback)
        self.goal_pub = self.create_publisher(PoseStamped, '/object_detector/goal_pose', 10)
        self.goal_pose_msg = None

        # -----------------------------------------------------

    def image_callback(self, color_msg, depth_msg, color_cam_info_msg):
        # convert the color and depth ROS2 image messages to OpenCV images
        # TODO: ------------- start --------------
        # in part 1,fill with your response
        #   you may need to nest things in a try, except in case frames are missing
        #.  if you are unpacking frames correctly, you should see the live color and depth output
        #   plotted in a cv2 window by detection_utils.visualize_detection_masks()
        # in part 2, you may need to make changes to the code to handle the head camera orientation


        try:
            self.latest_color = cv2.rotate(self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8'), cv2.ROTATE_90_CLOCKWISE)
            self.latest_depth = cv2.rotate(self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough'), cv2.ROTATE_90_CLOCKWISE)
            self.latest_color_cam_info = color_cam_info_msg


        except CvBridgeError as e:
            print(f"Error converting images: {e}")
        # TODO: -------------- end ---------------


    def publish_goals_callback(self):
        # run object detection on the RGB image
        # TODO: ------------- start --------------
        # fill with your response
        #   pass the color frame to YOLO-E, parse the results using detection_utils.parse_results()
        detections = None
        if self.latest_color is not None:
            results = self.model(self.latest_color, conf=0.25)
            detections = detection_utils.parse_results(results)

        # TODO: -------------- end ---------------

        # create visualizations from the detections
        if self.visualize and self.latest_color is not None:
            detection_utils.visualize_detections_masks(
                # TODO: minor - change the part= arg when you edit your code for part 2! 
                #   adjusts the color scaling of the depth image display to match the camera range
                part=2, detections=detections, rgb_image=self.latest_color, depth_image=self.latest_depth)

        # get the goal pose and publish it, if it exists
        self.get_goal_pose(detections)

        if self.goal_pose_msg is None:
            print("OBJECT NOT DETECTED, no pose to publish")
            return
    
        self.goal_pub.publish(self.goal_pose_msg)
        if detections is None or len(detections) == 0:
            print("(reusing last goal — no detection this frame)")
        print("---------- Published Goal Pose ----------")




    def get_goal_pose(self, detections, target_idx=0):
        if detections is None or len(detections) == 0:
            return None

        # TODO: ------------- start --------------
        # in part 1, fill with your response
        #   find the depth at the centroid and project it to 3D using detection_utils.pixel_to_3d()
        #   convert that pose to a PoseStamped msg using detection_utils.get_pose_msg()
        #   save that message to self.goal_pose_msg
        # in part 2, edit the code you wrote for part 1 to now project all points in the mask to 3D,
        #   then get the centroid of the resulting pointcloud to use as the goal pose (instead of the 2D centroid in part 1)
        # filter detections to only the target object if one is specified
        if self.target_obj is not None:
            for d in detections:
                print(repr(d['label']), "==", repr(self.target_obj), "?", d['label'] == self.target_obj)
            detections = [d for d in detections if d['label'] == self.target_obj]
            if len(detections) == 0:
                return
        target = detections[target_idx]
        mask_polygon = target['mask']  # Nx2 polygon vertices, NOT a binary mask

        # Rasterize the polygon into a binary mask image so we can iterate all interior pixels.
        # np.where directly on the polygon array gives wrong results (only col indices 0 or 1).
        h_rot, w_rot = self.latest_depth.shape[:2]
        mask_binary = np.zeros((h_rot, w_rot), dtype=np.uint8)
        cv2.fillPoly(mask_binary, [mask_polygon], 1)
        ys_rot, xs_rot = np.where(mask_binary)

        # camera_info intrinsics are for the ORIGINAL (unrotated) image.
        # After ROTATE_90_CLOCKWISE: (x_rot, y_rot) -> original (x_orig, y_orig) via:
        #   x_orig = y_rot,  y_orig = H_orig - 1 - x_rot
        h_orig = self.latest_color_cam_info.height

        points_3d = []
        for x_rot, y_rot in zip(xs_rot, ys_rot):
            depth = self.latest_depth[y_rot, x_rot]
            if depth > 0:
                x_orig = y_rot
                y_orig = h_orig - 1 - x_rot
                xyz = detection_utils.pixel_to_3d((x_orig, y_orig), depth, self.latest_color_cam_info)
                points_3d.append(xyz)
        
        print(f"mask pixels: {len(xs_rot)}, valid depth pixels: {len(points_3d)}")
        if len(points_3d) == 0:
            return
        centroid_3d = np.mean(points_3d, axis=0)
        if target['confidence'] < 0.7:
            print(f"Low confidence ({target['confidence']:.2f}) — reusing last goal pose")
            return 
        self.goal_pose_msg = detection_utils.get_pose_msg(
            self.latest_color_cam_info.header.stamp,
            self.latest_color_cam_info.header.frame_id,
            centroid_3d
        )
        # TODO: -------------- end ---------------


if __name__ == '__main__':
    rclpy.init()

    # load in the full list of object queries from the yaml file, as well as a target (if specified)
    with open('object_queries.yaml', 'r') as file:
        config = yaml.safe_load(file)
        obj_queries = config['queries']
        # read the target object to grasp (optional field in yaml)
        target_obj = config.get('target', None)

    yolo_object_detector = YOLOEObjectDetector(obj_queries, target_obj)
    rclpy.spin(yolo_object_detector)
    yolo_object_detector.destroy_node()
    rclpy.shutdown()
