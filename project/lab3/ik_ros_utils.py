import urchin as urdfpy
import importlib.resources as importlib_resources
import numpy as np
import ikpy.chain
from geometry_msgs.msg import Pose, PoseStamped
from tf2_geometry_msgs import TransformStamped

# lift up to table height, wrist yaw in line with base, wrist pitch slightly down, gripper open
READY_POSE_P1 = {
    'joint_lift': 0.8,
    'joint_wrist_yaw': 1.5,
    'joint_wrist_pitch': -0.1,
    'gripper_aperture': 0.5
}

READY_POSE_P2 = {
    'joint_lift': 0.8 ,
    'joint_arm_l0': 0.0,
    'joint_wrist_yaw': 0.5,
    'joint_wrist_pitch': -0.1,
    'gripper_aperture': 0.8,
    'joint_head_pan':-1.6,
    'joint_head_tilt':-0.5,
}

def get_xyz_from_msg(msg):

    if isinstance(msg, PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
    elif isinstance(msg, TransformStamped):
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

    return np.array((x, y, z))

def get_modified_urdf():
    pkg_path = str(importlib_resources.files('stretch_urdf'))
    urdf_file_path = pkg_path + '/SE3/stretch_description_SE3_eoa_wrist_dw3_tool_sg3.urdf'

    # Remove unnecessary links/joints
    original_urdf = urdfpy.URDF.load(urdf_file_path)
    modified_urdf = original_urdf.copy()

    names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'gripper_camera_bottom_screw_frame', 'gripper_camera_link', 'gripper_camera_depth_frame', 'gripper_camera_depth_optical_frame', 'gripper_camera_infra1_frame', 'gripper_camera_infra1_optical_frame', 'gripper_camera_infra2_frame', 'gripper_camera_infra2_optical_frame', 'gripper_camera_color_frame', 'gripper_camera_color_optical_frame', 'laser', 'base_imu', 'respeaker_base', 'link_wrist_quick_connect', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_aruco_fingertip_right', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_aruco_fingertip_left', 'link_aruco_d405', 'link_head_nav_cam']
    # links_kept = ['base_link', 'link_mast', 'link_lift', 'link_arm_l4', 'link_arm_l3', 'link_arm_l2', 'link_arm_l1', 'link_arm_l0', 'link_wrist_yaw', 'link_wrist_yaw_bottom', 'link_wrist_pitch', 'link_wrist_roll', 'link_gripper_s3_body', 'link_grasp_center']
    links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
    for lr in links_to_remove:
        modified_urdf._links.remove(lr)
    names_of_joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'gripper_camera_joint', 'gripper_camera_link_joint', 'gripper_camera_depth_joint', 'gripper_camera_depth_optical_joint', 'gripper_camera_infra1_joint', 'gripper_camera_infra1_optical_joint', 'gripper_camera_infra2_joint', 'gripper_camera_infra2_optical_joint', 'gripper_camera_color_joint', 'gripper_camera_color_optical_joint', 'joint_laser', 'joint_base_imu', 'joint_respeaker', 'joint_wrist_quick_connect', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_aruco_fingertip_right', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_aruco_fingertip_left', 'joint_aruco_d405', 'joint_head_nav_cam']
    # joints_kept = ['joint_mast', 'joint_lift', 'joint_arm_l4', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_yaw_bottom', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_gripper_s3_body', 'joint_grasp_center']
    joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
    for jr in joints_to_remove:
        modified_urdf._joints.remove(jr)

    # TODO: ------------- start --------------
    # fill with your response
    #   your implementation from lab 2 - add a virtual base rotation joint to the urdf
    link_base_rotation = urdfpy.Link(name='link_base_rotation',
                                     inertial=None,
                                     visuals=None,
                                     collisions=None)
    modified_urdf._links.append(link_base_rotation)
    joint_base_rotation = urdfpy.Joint(name='joint_base_rotation',
                                       parent='base_link',
                                       child='link_base_rotation',
                                       joint_type='revolute',
                                       axis=np.array([0.0, 0.0, 1.0]),
                                       origin=np.eye(4, dtype=np.float64),
                                       limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-0.5, upper=0.5))
    modified_urdf._joints.append(joint_base_rotation)
    # TODO: -------------- end ---------------

    joint_base_translation = urdfpy.Joint(name='joint_base_translation',
                                        parent='link_base_rotation',
                                        child='link_base_translation',
                                        joint_type='prismatic',
                                        axis=np.array([1.0, 0.0, 0.0]),
                                        origin=np.eye(4, dtype=np.float64),
                                        limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-0.5, upper=0.5))
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

    new_urdf_path = "/tmp/iktutorial/stretch.urdf"
    modified_urdf.save(new_urdf_path)
    return new_urdf_path

new_urdf_path = get_modified_urdf()
chain = ikpy.chain.Chain.from_urdf_file(new_urdf_path)

for link in chain.links:
    print(f"* Link Name: {link.name}, Type: {link.joint_type}")

def get_current_configuration(joint_state):
    # TODO: ------------- start --------------
    # fill with your response
    #   your implementation from lab 2 - get the current configuration from the joint state
    #   note: this time you can use the joint state callback provided for you in target_following.py which provides joint states as a
    #   dictionary that can be indexed by joint name, e.g. joint_state['joint_lift']
    def bound_range(name, value):
        names = [l.name for l in chain.links]
        index = names.index(name)
        bounds = chain.links[index].bounds
        return min(max(value, bounds[0]), bounds[1])

    q_rotation = 0.0
    q_translation = 0.0
    q_lift = bound_range('joint_lift', joint_state['joint_lift'])
    q_arml = bound_range('joint_arm_l0', joint_state['joint_arm_l0'])
    q_yaw = bound_range('joint_wrist_yaw', joint_state['joint_wrist_yaw'])
    q_pitch = bound_range('joint_wrist_pitch', joint_state['joint_wrist_pitch'])
    q_roll = bound_range('joint_wrist_roll', joint_state['joint_wrist_roll'])
    q = [0.0, q_rotation, q_translation, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]

    return q
    # TODO: -------------- end ---------------

def get_current_grasp_pose():
    q = get_current_configuration()
    return chain.forward_kinematics(q)

def get_grasp_goal(target_point, target_orientation, q_init):
    # previously the move_to_grasp() function from lab 2
    #   moved to it's own function without the final move_to_configuration() call for convenience in this lab
    q_soln = chain.inverse_kinematics(target_point, target_orientation, orientation_mode=None, initial_position=q_init)
    # print('Solution:', q_soln)
    print("Solution Found")

    err = np.linalg.norm(chain.forward_kinematics(q_soln)[:3, 3] - target_point)
    if not np.isclose(err, 0.0, atol=3e-2):
        print("IKPy did not find a valid solution")
        return
    # move_to_configuration(q=q_soln)
    return q_soln

def move_to_configuration(node, q):
    # TODO: ------------- start --------------
    # fill with your response
    #   your implementation from lab 2 - unpack the q solution to appropriate ros2 joints and command the robot joints to move accordingly
    q_rotation = q[1]
    q_translation = q[2]
    q_lift = q[4]
    q_arm = q[6] + q[7] + q[8] + q[9]
    q_yaw = q[10]
    q_pitch = q[12]
    q_roll = q[13]
    # Move lift first to correct height, then extend arm so the arm doesn't collide
    node.move_to_pose({'joint_lift': q_lift}, blocking=True)
    node.move_to_pose({
        'joint_arm': q_arm,
        'joint_wrist_yaw': q_yaw,
        'joint_wrist_pitch': q_pitch,
        'joint_wrist_roll': q_roll
    })
    # Only move base if displacement is large enough to avoid camera losing sight of object
    if abs(q_rotation) > 0.1:   # threshold: ~6 degrees
        node.move_to_pose({'rotate_mobile_base': q_rotation})
    if abs(q_translation) > 0.05:  # threshold: 5 cm
        node.move_to_pose({'translate_mobile_base': q_translation})
    # TODO: -------------- end ---------------

def print_q(q):
    if q is None:
        print('INVALID Q')

    else:
        print("IK Config")
        print("     Base Rotation:", q[1])
        print("     Base Translation:", q[2])
        print("     Lift", q[4])
        print("     Arm", q[6] + q[7] + q[8] + q[9])
        print("     Gripper Yaw:", q[10])
        print("     Gripper Pitch:", q[12])
        print("     Gripper Roll:", q[13])
