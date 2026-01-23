import time
import numpy as np
import rclpy
import tf2_ros
import hello_helpers.hello_misc as hm
node = hm.HelloNode.quick_create('lab1_ros2')

# task begin

node.stow_the_robot()
time.sleep(1.0)

node.move_to_pose({'joint_arm':0.5,
                  'joint_lift':1.1},blocking = True)

node.move_to_pose({'joint_wrist_yaw':np.radians(8)},blocking = True)

node.move_to_pose({'joint_wrist_pitch':np.radians(8)},blocking = True)

node.move_to_pose({'joint_wrist_roll':np.radians(20)},blocking = True)

node.move_to_pose({'joint_gripper_finger_left': 0.6}, blocking=True)

node.move_to_pose({'joint_gripper_finger_right': -0.3}, blocking=True)

node.move_to_pose({'joint_head_pan': np.radians(45)}, 
                  blocking=True)  
node.move_to_pose({'joint_head_tilt': np.radians(-45)}, 
                  blocking=True) #note if i assigned a positive number the tilt motion wont move which i am not sure the reason？
node.stow_the_robot()
time.sleep(1.0)

node.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)

node.move_to_pose({'rotate_mobile_base': np.radians(180)}, blocking=True)


node.move_to_pose({'translate_mobile_base': 0.5}, blocking=True)

print("task completed")
node.destroy_node()
rclpy.shutdown()
