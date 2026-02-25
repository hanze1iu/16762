# 16762-lab3

## Lab 3: Overview

In this lab, you'll be asked to build on the inverse kinematics concepts you explored in Lab 2, this time using visual perception to identify, track, and grasp objects of interest, implemented using ROS2!

We'll be leveraging [YOLO-E](https://docs.ultralytics.com/models/yoloe/) as part of our visual perception module, enabling zero-shot, real-time object detection and segmentation based on a text prompt.

## Install Dependencies
### Dependencies for using YOLO-E
```
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install -U ultralytics
```
Of the available versions of YOLO-E, we will be using `yolo-e-v26-small` for its great performance and inference latency on limited compute, perfect for edge applications (e.g., running directly on the Stretch CPU). The pretrained weights should already be downloaded on Stretch for you and saved to: `/home/hello-robot/models/yoloe-26s-seg.pt`. Should you need to download them again for some reason, they can be found [here](https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt).

You may see something like this the first time you try to run the model:
```
WARNING ⚠️ Ultralytics settings reset to default values. This may be due to a possible problem with your settings or a recent ultralytics package update
```
Don't worry, it will automatically resolve any issues and download/install whatever is missing.


### Dependencies for IK solving (same as Lab 2)
```
pip install --upgrade ikpy graphviz urchin networkx
```

## Part 1: Position-Based Visual Servoing

**GOAL: Detect a target object in the scene using the in-gripper camera and have the robot move both its base and gripper to follow it!**

You will primarily be editing two starter scripts, corresponding to different ros2 nodes, that handle object detection/segmentation and IK for object following, respectively. For more details on what each script does and what you should implement:

### `object_detector.py`

1. (Provided) Loads in `yolo-e-v26-small`.
2. TODO: populate `object_queries.yaml` with the object/s you want the robot to be able to detect
3. (Provided) Creates a synchronized subscriber to take in color and aligned-depth frames from the in-gripper camera, as well as the camera intrinsics.
4. TODO: implement a image callback function to unpack the incoming color and depth frames (cv2bridge). Once this is implemented correctly, you should see the live color and depth output plotted in a cv2 window by `detection_utils.visualize_detection_masks()` (without any masks yet thoughh as you'll implement that next)
5. TODO: Implement a timer callback function feed color frames into YOLO-E at a fixed frequency.
6. TODO: Pass the results from YOLO-E to `detection_utils.parse_results()` to extract the detections (bounding, boxes, masks).
7. TODO: For each detected item in the color image, find the depth of the centroid of the object mask and project it to 3D using `detection_utils.pixel_to_3d()`.
8. TODO: Convert the 3D position of the goal to a PoseStamped msg
9. (Provided) Publish pose message as the goal point for the robot to track.

This script uses a number of helper functions that we provide in `detection_utils.py`. You may need to modify some of those helper functions to match your specific implementation.

Among one of those helper functions is a visualization tool (using cv2, mentioned in step 4) which will plot the color and depth frames, adding annotations for detected objects as you get those parts of your code working. This should hopefully help with debugging! You may notice that there's some lag in the image plotting - this is expected.

### `target_following.py`

1. TODO: Run the code once before making any changes. The robot should stow itself, then move to its ready configuration, which is designed such that: the lift is up at ~table height, wrist yaw is in line with the base, pitch is slightly downward, gripper is open. Adjust initial lift height and pitch as needed for your workspace (`READY_POSE_P1` found in `ik_ros_utils.py`).
2. (Provided) Creates a joint state callback that unpacks joint state messages for what works with/is expected by ikpy
3. TODO: subscribe to the `object_detector/goal_pose` topic in `self.main()`, published by your object detection node, to find the target pose the robot should move to.
4. TODO: Also create a TF buffer and listener in `self.main()`
5. TODO: Transform the goal pose to the same coordinate system as the robot's base
6. TODO: Transform the gripper pose to the same coordinate system as the robot's base
7. TODO: If the goal is too far to move to before the next goal is posted (~2Hz), compute a waypoint some delta away from the current gripper position to make progress to the goal
8: TODO: Use the same functions you used for IK in Lab 2, provided for you now in `ik_ros_utils.py`, to move the robot to the waypoint.

When both nodes are working, you should take a video of the robot following the object to 3 different positions of the object. The robot's base should have to move to track the object to at least one of those poses.

## Part 2: 3D Perception for Grasping

**GOAL: Detect a target object in the scene using the head camera and have the robot move to grasp it!**

This part of the lab depends heavily on the previous part and involves editing those files for this new task.

### `object_detector.py` --> copy and rename `object_detector_pcd.py`

1. TODO: Modify color, depth, and cam info subscribers/callback to use the head camera instead of the in-gripper one.
2. TODO: Add 3-4 more candidates to grasp to `object_queries.yaml`. You should select objects with interesting variations in geometry or physical properties.
3. TODO (optional): if you are having a lot of trouble with missing depth values near where the centroid of the object is (common for reflective objects), Add another step of processing that fills in missing depth values across the mask of the detected object.
4. TODO: Edit `get_goal_pose()` to project all the points in the mask to 3D, then compute the centroid of that resulting pointcloud to get the goal/grasp point

### `target_following.py` --> copy and rename `grasp_objects.py`

1. Change the ready pose by uncommenting the line for part 2. Run the code once before making any further changes. The robot should stow itself, then move to its ready configuration, which is designed such that: the lift is up at ~table height, wrist yaw is normal to the base, pitch is slightly downward, gripper is open, and head is looking down the arm. Adjust initial lift height and pitch as needed for your workspace (`READY_POSE_P1` found in `ik_ros_utils.py`).
2. TODO: Edit the code to move to and grasp a object (instead of continuously following it). You should be able to pick up a given object and retract the arm to bring it closer to the base. 
3. TODO: Rerun for all the objects.




# Notes/Troubleshooting
- Avoid reflective objects as the target as these can give you poor depth values. See the `object_queries.yaml` file for more notes on selecting objects to detect.

- CV2 expects color frames in BGR, YOLO-E expects them in RGB format. Further, YOLO outputs in pixel coordinates, while array indexing is the opposite - this can trip you up!

- Cameras can go down, seemingly at random. If you find your code randomly stops working, make sure to check the terminals running the cameras.

- If you have to run-stop the robot, you might want to restart the stretch driver and camera nodes - handling the robot can mess up where the robot thinks it is in the world, which can introduce error in your transforms.

- While we provide a tool for visualizing the color and depth frames with the detected object masks, Rviz is your friend when it comes to visualizing things in 3D (goal poses, transforms, etc.)! For example, you might use Rviz to verify where your published goal pose is with respect to where you expect it to be in the pointcloud, to make sure your detection is working well! You should be using it often, especially when implementing the nodes that move the robot.
