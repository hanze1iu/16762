import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from hello_helpers.hello_misc import HelloNode
import threading
import tf2_ros
from tf2_geometry_msgs import TransformStamped
from sensor_msgs.msg import JointState
import ik_ros_utils as ik
import ikpy

# Make sure to run:
#   ros2 launch stretch_core stretch_driver.launch.py

class IKTargetFollowing(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)

        self.delta = 0.03 # cm
        self.target_frame = 'base_link'
        self.gripper_frame = 'link_grasp_center'
        self.tf_buffer = None
        self.tf_listener = None
        self.joint_states_lock = threading.Lock()
    
    def joint_states_callback(self, msg):
        # unpacks joint state messages for what works with/is expected by ikpy
        with self.joint_states_lock:
            joint_states = msg
        # extract information needed for ik_solver
        joint_names = [
            'joint_lift', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'
        ]
        self.joint_state = {}
        for joint_name in joint_names:
            i = joint_states.name.index(joint_name)
            self.joint_state[joint_name] = joint_states.position[i]

    def get_goal_pose_in_base_frame(self, goal_msg):
        # TODO: ------------- start --------------
        # fill with your response
        #   transform the goal pose to the base frame

        goal_transformed = None
        # TODO: -------------- end ---------------

        return goal_transformed

    def get_gripper_pose_in_base_frame(self):
        # TODO: ------------- start --------------
        # fill with your response
        #   transform the gripper pose to the base frame

        gripper_transformed = None
        # TODO: -------------- end ---------------

        return gripper_transformed

    def goal_callback(self, goal_msg):
        # print(msg)
        # self.get_logger().info(f'Received goal pose: {msg.pose}')
        try:
            goal_transformed = self.get_goal_pose_in_base_frame(goal_msg)
            gripper_transformed = self.get_gripper_pose_in_base_frame()

            goal_pos = ik.get_xyz_from_msg(goal_transformed)
            gripper_pos = ik.get_xyz_from_msg(gripper_transformed)
        except:
            print("Error getting transforms")
            return

        waypoint_pos, waypoint_orient = self.compute_waypoint_to_goal(goal_pos, gripper_pos)

        # TODO: ------------- start --------------
        # fill with your response
        #   use the same functions you used for IK in Lab 2, now in `ik_ros_utils.py`, 
        #   to move the robot to the transformed goal point.
        q_soln = None
        # TODO: -------------- end ---------------

        # NOTE: if you find that the robot's base is moving too much, its likely that the ik solver is
        # struggling to find solutions without the base doing most of the work to achieve the waypoint pose.
        # you can adjust the `self.delta` variable to be smaller so that the displacements are smaller, and
        #   there is a valid solution without excessive base movement
        # you can also set your own triggers manually (keep delta large but use an if/else on move_to_pose()
        #   so the base only moves above a certain distance threshold
        # you can also try adjusting joint limits of the base trans/rot in `ik_ros_utils.py` to be much smaller
        # one or some combination of these should help!

        ik.print_q(q_soln)
        if q_soln is not None:
            ik.move_to_configuration(self, q_soln)

    def compute_waypoint_to_goal(self, goal_pos, gripper_pos):

        # TODO: ------------- start --------------
        # fill with your response
        #   find the distance between the published goal position and the gripper position
        #   if its above some threshold (delta), consider the goal to be too far (since we're trying to track the object
        #   at least 2Hz) to reach before the next goal is published
        #   in this case, find a waypoint toward the goal position that is delta away from the gripper position (make some progress towards the goal)
        #   otherwise, the goal is close and we can move there directly

        waypoint_pos = None
        # TODO: -------------- end ---------------

        # use an zero rotation for the waypoint (its a point so we don't need to worry about orientation)
        waypoint_orient = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0) # [roll, pitch, yaw]

        return waypoint_pos, waypoint_orient


    def move_to_ready_pose(self):
        # TODO: minor - uncomment the correct ready pose for part 1 or 2!
        #   part 1: 
        self.move_to_pose(ik.READY_POSE_P1, blocking=True)
        #   part 2: READY_POSE_P2
        # self.move_to_pose(ik.READY_POSE_P2, blocking=True)

    def main(self):
        HelloNode.main(self, 'follow_target', 'follow_target', wait_for_first_pointcloud=False)
        self.logger = self.get_logger()
        self.callback_group = ReentrantCallbackGroup()
        self.joint_states_subscriber = self.create_subscription(JointState, '/stretch/joint_states', callback=self.joint_states_callback, qos_profile=1)

        self.stow_the_robot()
        self.move_to_ready_pose()
        print("At Ready Pose")


        # TODO: ------------- start --------------
        # fill with your response
        #   create a tf2 buffer and listener
        #   create a subscriber to the goal pose published by your object detector
        # TODO: -------------- end ---------------






if __name__ == '__main__':
    target_follower = IKTargetFollowing()
    target_follower.main()
    target_follower.new_thread.join()
