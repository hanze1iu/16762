# ros2 launch stretch_core stretch_driver.launch.py

import ikpy.urdf.utils
import urchin as urdfpy
import numpy as np
import ikpy.chain
import hello_helpers.hello_misc as hm
import importlib.resources as importlib_resources

# NOTE before running: `python3 -m pip install --upgrade ikpy graphviz urchin networkx`

def setup_ik_chain():
    """Setup the IK chain from URDF with virtual base joint."""
    pkg_path = str(importlib_resources.files('stretch_urdf'))
    urdf_file_path = pkg_path + '/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf'

    # Remove unnecessary links/joints
    original_urdf = urdfpy.URDF.load(urdf_file_path)
    modified_urdf = original_urdf.copy()

    names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'gripper_camera_bottom_screw_frame', 'gripper_camera_link', 'gripper_camera_depth_frame', 'gripper_camera_depth_optical_frame', 'gripper_camera_infra1_frame', 'gripper_camera_infra1_optical_frame', 'gripper_camera_infra2_frame', 'gripper_camera_infra2_optical_frame', 'gripper_camera_color_frame', 'gripper_camera_color_optical_frame', 'laser', 'base_imu', 'respeaker_base', 'link_wrist_quick_connect', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_aruco_fingertip_right', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_aruco_fingertip_left', 'link_aruco_d405', 'link_head_nav_cam']
    links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
    for lr in links_to_remove:
        modified_urdf._links.remove(lr)
    # links_kept = ['base_link', 'link_mast', 'link_lift', 'link_arm_l4', 'link_arm_l3', 'link_arm_l2', 'link_arm_l1', 'link_arm_l0', 'link_wrist_yaw', 'link_wrist_yaw_bottom', 'link_wrist_pitch', 'link_wrist_roll', 'link_gripper_s3_body', 'link_grasp_center']

    names_of_joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'gripper_camera_joint', 'gripper_camera_link_joint', 'gripper_camera_depth_joint', 'gripper_camera_depth_optical_joint', 'gripper_camera_infra1_joint', 'gripper_camera_infra1_optical_joint', 'gripper_camera_infra2_joint', 'gripper_camera_infra2_optical_joint', 'gripper_camera_color_joint', 'gripper_camera_color_optical_joint', 'joint_laser', 'joint_base_imu', 'joint_respeaker', 'joint_wrist_quick_connect', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_aruco_fingertip_right', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_aruco_fingertip_left', 'joint_aruco_d405', 'joint_head_nav_cam'] 
    joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
    for jr in joints_to_remove:
        modified_urdf._joints.remove(jr)
    # joints_kept = ['joint_mast', 'joint_lift', 'joint_arm_l4', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_yaw_bottom', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_gripper_s3_body', 'joint_grasp_center']

    # Add virtual base joint
    joint_base_rotation = urdfpy.Joint(name='joint_base_rotation',
                                          parent='base_link',
                                          child='link_base_rotation',
                                          joint_type='revolute',
                                          axis=np.array([0.0, 0.0, 1.0]),
                                          origin=np.eye(4, dtype=np.float64),
                                          limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-np.pi, upper=np.pi))
    modified_urdf._joints.append(joint_base_rotation)

    link_base_rotation = urdfpy.Link(name='link_base_rotation',
                                        inertial=None,
                                        visuals=None,
                                        collisions=None)
    modified_urdf._links.append(link_base_rotation)

    joint_base_translation = urdfpy.Joint(name='joint_base_translation',
                                          parent='link_base_rotation',
                                          child='link_base_translation',
                                          joint_type='prismatic',
                                          axis=np.array([1.0, 0.0, 0.0]),
                                          origin=np.eye(4, dtype=np.float64),
                                          limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-1.0, upper=1.0))
    modified_urdf._joints.append(joint_base_translation)
    link_base_translation = urdfpy.Link(name='link_base_translation',
                                        inertial=None,
                                        visuals=None,
                                        collisions=None)
    modified_urdf._links.append(link_base_translation)


    
    # amend the chain
    for j in modified_urdf._joints:
        if j.name == 'joint_mast':
            j.parent = 'link_base_translation'

    import os
    os.makedirs('/tmp/iktutorial', exist_ok=True)
    new_urdf_path = "/tmp/iktutorial/stretch.urdf"
    modified_urdf.save(new_urdf_path)
    active_links_mask = [
    False,  # 0: Base link (fixed)
    True,   # 1: joint_base_rotation (revolute)
    True,   # 2: joint_base_translation (prismatic)
    False,  # 3: joint_mast (fixed) 
    True,   # 4: joint_lift (prismatic) 
    False,  # 5: joint_arm_l4 (fixed)
    True,   # 6: joint_arm_l3 (prismatic) 
    True,   # 7: joint_arm_l2 (prismatic) 
    True,   # 8: joint_arm_l1 (prismatic) 
    True,   # 9: joint_arm_l0 (prismatic) 
    True,   # 10: joint_wrist_yaw (revolute) 
    False,  # 11: joint_wrist_yaw_bottom (fixed)
    True,   # 12: joint_wrist_pitch (revolute) 
    True,   # 13: joint_wrist_roll (revolute) 
    False,  # 14: joint_gripper_s3_body (fixed)
    False,  # 15: joint_grasp_center (fixed)]
    ]
    chain = ikpy.chain.Chain.from_urdf_file(new_urdf_path, active_links_mask=active_links_mask)
    
    for link in chain.links:
        print(f"* Link Name: {link.name}, Type: {link.joint_type}")
    
    return chain


class StretchIKDemo(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        self.chain = None

    def bound_range(self, name, value):
        """Bound value to joint limits."""
        names = [l.name for l in self.chain.links]
        index = names.index(name)
        bounds = self.chain.links[index].bounds
        return min(max(value, bounds[0]), bounds[1])

    def get_current_configuration(self):
        """Get current robot configuration from ROS joint states."""
        # Get joint positions from HelloNode's joint_state
        joint_positions = self.joint_state.position
        joint_names = self.joint_state.name
        
        def get_joint_pos(name):
            if name in joint_names:
                idx = list(joint_names).index(name)
                return joint_positions[idx]
            return 0.0

        q_base_rotate = 0.0
        q_base_translate = 0.0
        q_lift = self.bound_range('joint_lift', get_joint_pos('joint_lift'))
        q_arml = self.bound_range('joint_arm_l0', get_joint_pos('joint_arm_l0'))
        q_yaw = self.bound_range('joint_wrist_yaw', get_joint_pos('joint_wrist_yaw'))
        q_pitch = self.bound_range('joint_wrist_pitch', get_joint_pos('joint_wrist_pitch'))
        q_roll = self.bound_range('joint_wrist_roll', get_joint_pos('joint_wrist_roll'))
        
        return [0.0, q_base_rotate, q_base_translate, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]
        # base_link, link_base_rotation, link_base_translation, link_mast, link_lift, link_arm_l4, link_arm_l3, link_arm_l2, link_arm_l1, link_arm_l0, link_wrist_yaw, link_wrist_yaw_bottom, link_wrist_pitch, link_wrist_roll, link_gripper_s3_body, link_grasp_center
    
    def move_to_configuration(self, q):
        """Move robot to IK solution configuration using ROS."""
        q_base_rotate = q[1]
        q_base_translate = q[2]
        q_lift = q[4]
        q_arm = q[6] + q[7] + q[8] + q[9] 
        q_yaw = q[10]
        q_pitch = q[12]
        q_roll = q[13]
        
        self.move_to_pose(
            {
                'joint_lift': q_lift,
                'joint_arm': q_arm,
                'joint_wrist_yaw': q_yaw,
                'joint_wrist_pitch': q_pitch,
                'joint_wrist_roll': q_roll,
            },
            blocking=True,
            duration=3.0
        )
        self.move_to_pose({'rotate_mobile_base': q_base_rotate}, blocking=True, duration=4.0)
        self.move_to_pose({'translate_mobile_base': q_base_translate}, blocking=True, duration=4.0)


    def move_to_grasp_goal(self, target_point, target_orientation):
        """Compute IK and move to grasp goal."""
        q_init = self.get_current_configuration()
        q_soln = self.chain.inverse_kinematics(
            target_point, 
            target_orientation, 
            orientation_mode='all', 
            initial_position=q_init
        )
        print('Solution:', q_soln)

        err = np.linalg.norm(self.chain.forward_kinematics(q_soln)[:3, 3] - target_point)
        if not np.isclose(err, 0.0, atol=1e-2):
            print("IKPy did not find a valid solution")
            return None
        
        self.move_to_configuration(q=q_soln)
        return q_soln

    def get_current_grasp_pose(self):
        q = self.get_current_configuration()
        return self.chain.forward_kinematics(q)

    def main(self):
        hm.HelloNode.main(
            self,
            node_name='stretch_ik_demo',
            node_topic_namespace='stretch_ik_demo',
            wait_for_first_pointcloud=False
        )
        
        self.stow_the_robot()
        # Setup IK chain
        self.chain = setup_ik_chain()
        # for i, link in enumerate(self.chain.links):
        #     print(f"{i}: {link.name}, type={link.joint_type}, bounds={link.bounds}")
            
        # Define target pose
        # target_point = [0.5, 0.3, 0.1]
        # target_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, -np.pi/2)  # [roll, pitch, yaw]       
        # # Move to grasp goal
        # self.move_to_grasp_goal(target_point, target_orientation)
        # print(self.get_current_grasp_pose())
        print("===4_steps_goal===")
        z_walk_poses = [
        ([0.6, -0.3, 0.4], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        ([0.5, 0.3, 0.2], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        ([0.6, -0.3, 0.4], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),
        ([0.5, 0.3, 0.2], ikpy.utils.geometry.rpy_matrix(0, 0, 0)),]
        for i, (point, orientation) in enumerate(z_walk_poses):
            self.move_to_grasp_goal(point, orientation)
        
        # print("===z_draw===")
        # z_draw_poses = [
        # ([0.4, -0.3, 0.9], ikpy.utils.geometry.rpy_matrix(-np.pi/2, 0, 0)),
        # ([0.6, -0.3, 0.9], ikpy.utils.geometry.rpy_matrix(-np.pi/2, 0, 0)),
        # ([0.4, -0.3, 0.4], ikpy.utils.geometry.rpy_matrix(-np.pi/2, 0, 0)),
        # ([0.6, -0.3, 0.4], ikpy.utils.geometry.rpy_matrix(-np.pi/2, 0, 0)),]
        # for i, (point, orientation) in enumerate(z_draw_poses):
        #     self.move_to_grasp_goal(point, orientation)
  
        # Stop
        self.stop_the_robot()


if __name__ == '__main__':
    node = StretchIKDemo()
    node.main()
