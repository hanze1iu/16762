import rclpy, time
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from hello_helpers.hello_misc import HelloNode
import moveit2_utils

# Make sure to run `ros2 launch stretch_core stretch_driver.launch.py`

class MoveMe(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        self.main('move_me', 'move_me', wait_for_first_pointcloud=False)
        self.stow_the_robot()

        planning_group = 'mobile_base_arm'
        moveit, moveit_plan, planning_params = moveit2_utils.setup_moveit(planning_group)
        
        for i in range(1):
            print(f'--- Planning Step {i} ---')
            goal_state = RobotState(moveit.get_robot_model())

            # Example goal state: demonstrates commanding the robot to move forward by 0.3 meters
            # We pass in the current joint position as the goal for each joint in the arm + wrist since we don't want them to move
            # Ordering: [x, y, theta, lift, arm/4, arm/4, arm/4, arm/4, yaw, pitch, roll]
            # For driving the base: the positive x-axis is pointing out of the front of the robot (the flat side of the base). 
            # Positive y-axis is on the left of the robot (opposite direction the arm is facing).
            goal_state.set_joint_group_positions(planning_group, 
                [0.3, 0.0, 0.0, 
                self.get_joint_pos('joint_lift'), self.get_joint_pos('joint_arm_l3'), 
                self.get_joint_pos('joint_arm_l2'), self.get_joint_pos('joint_arm_l1'), self.get_joint_pos('joint_arm_l0'), 
                self.get_joint_pos('joint_wrist_yaw'), self.get_joint_pos('joint_wrist_pitch'), self.get_joint_pos('joint_wrist_roll')]
            )
            # NOTE: You should delete the above example and replace it with your own goal states.
            
            # TODO: Your code will likely go here. Note that I gave you a for loop already, which you can edit and use.
            # For base motion (x, y, theta) each goal state should be defined according to the robot's current position (or its previous goal).
            # Reminder: You can use the RViz GUI for MoveIt 2 to get a better intuition for what these goal positions should be.

            moveit_plan.set_start_state_to_current_state()
            moveit_plan.set_goal_state(robot_state=goal_state)
            
            plan = moveit_plan.plan(parameters=planning_params)
            # print(plan.trajectory.get_robot_trajectory_msg())
    
            self.execute_plan(plan)

    def execute_plan(self, plan):
        # NOTE: You don't need to edit this function
        processor = moveit2_utils.TrajectoryProcessor()
        segments = processor.process_trajectory(plan, self.joint_state)

        for i, goal_traj in enumerate(segments):
            # print(goal_traj)
            # time.sleep(2.0)
            self.get_logger().info(f"Executing segment {i+1}/{len(segments)} (Mode: {self._detect_mode(goal_traj)})")
            
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = goal_traj
            
            future = self.trajectory_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error(f"Segment {i+1} rejected!")
                break
            
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            res = res_future.result()
            
            if res.result.error_code != res.result.SUCCESSFUL:
                self.get_logger().error(f"Segment {i+1} failed! Code: {res.result.error_code}")
                break

    def get_joint_pos(self, joint_name):
        return self.joint_state.position[self.joint_state.name.index(joint_name)]
        
    def _detect_mode(self, traj):
        if 'translate_mobile_base' in traj.joint_names: return 'TRANSLATE'
        if 'rotate_mobile_base' in traj.joint_names: return 'ROTATE'
        return 'ARM_ONLY'

if __name__ == '__main__':
    MoveMe()
