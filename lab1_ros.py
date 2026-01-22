import time
import numpy as np
import rclpy
import tf2_ros
import hello_helpers.hello_misc as hm
node = hm.HelloNode.quick_create('lab1_ros2')

# task begin

node.stow_the_robot()
time.sleep(2.0)

node.move_to_pose({'joint_arm':0.5,
                  'joint_lift':1.1},blocking = True)

node.move_to_pose({'joint_wrist_yaw':np.radians(30)},blocking = True)

node.move_to_pose({'joint_wrist_pitch':np.radians(30)},blocking = True)

node.move_to_pose({'joint_wrist_roll':np.radians(30)},blocking = True)

node.move_to_pose({'joint_gripper_finger_left': 0.6}, blocking=True)

node.move_to_pose({'joint_gripper_finger_right': -0.6}, blocking=True)

current_pan = node.joint_state.position[node.joint_state.name.index('joint_head_pan')]
node.move_to_pose({'joint_head_pan': current_pan + np.radians(45)}, blocking=True)

current_tilt = node.joint_state.position[node.joint_state.name.index('joint_head_tilt')]
node.move_to_pose({'joint_head_tilt': current_tilt + np.radians(45)}, blocking=True)

node.stow_the_robot()
time.sleep(2.0)

node.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)

node.move_to_pose({'rotate_mobile_base': np.radians(180)}, blocking=True)


node.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)

print("task completed")