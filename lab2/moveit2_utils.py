import yaml, copy
import numpy as np
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import tf_transformations as tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def create_collision_object(moveit, name, frame_id, shape_type, dimensions, position, orientation=(0.0, 0.0, 0.0, 1.0)):
    """
    frame_id: which link on the robot is this object's position/orientation defined relative to?
    shape_type: SolidPrimitive.BOX, ... BOX, SPHERE, CYLINDER, CONE, PRISM
    dimensions: list (e.g., [x, y, z] for BOX, [h, r] for CYLINDER)
    position: list [x, y, z]
    """
    co = CollisionObject()
    co.id = name
    co.header.frame_id = frame_id
    
    # Define the shape
    primitive = SolidPrimitive()
    primitive.type = shape_type
    primitive.dimensions = dimensions
    co.primitives.append(primitive)
    
    # Define the pose
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    co.primitive_poses.append(pose)
    
    co.operation = CollisionObject.ADD

    planning_scene_monitor = moveit.get_planning_scene_monitor()
    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(co)
        scene.current_state.update()

    return co

def setup_moveit(planning_group='mobile_base_arm'):
    moveit_config_dict = (
        MoveItConfigsBuilder('stretch3_description_dex', package_name='stretch_moveit2')
        .trajectory_execution(file_path='config/ros_controllers.yaml')
        .sensors_3d(file_path='config/sensors_4d.yaml')
        .pilz_cartesian_limits(file_path='config/joint_limits.yaml')
        .moveit_cpp(file_path='config/moveit_cpp.yaml') 
        .trajectory_execution(file_path='config/trajectory_execution.yaml', moveit_manage_controllers=True)
        .joint_limits(file_path='config/joint_limits.yaml')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    ).to_dict()

    moveit_controllers = {
        'controller_names': ['stretch_controller'],
        'stretch_controller': {
            'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'allow_partial_joints_goal': True,
            'action_ns': 'follow_joint_trajectory',
            'type': 'FollowJointTrajectory',
            'default': True,
            'joints': ['joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_head_pan', 'joint_head_tilt', 'joint_gripper_finger_left', 'joint_gripper_finger_right', 'position']
        }
    }
    
    moveit_config_dict.update({
        'stretch_simple_controller_manager': moveit_controllers,
        'moveit_manage_controllers': True,
        'moveit_controller_manager': 'stretch_simple_controller_manager/MoveItSimpleControllerManager',
    })

    if 'planning_pipelines' not in moveit_config_dict:
        moveit_config_dict['planning_pipelines'] = {}
        
    moveit_config_dict['planning_pipelines']['ompl'] = {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization
                               default_planner_request_adapters/FixWorkspaceBounds
                               default_planner_request_adapters/FixStartStateBounds
                               default_planner_request_adapters/FixStartStateCollision
                               default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 2.0,  # Relaxed tolerance (meters/radians)
        'jiggle_fraction': 0.05
    }
    
    moveit_config_dict['trajectory_execution'] = {
        'allowed_execution_duration_scaling': 2.0,
        'allowed_goal_duration_margin': 5.0,
        'allowed_start_tolerance': 0.1,
    }

    moveit_config_dict['robot_description_kinematics'] = {}
    moveit_config_dict['robot_description_kinematics']['mobile_base_arm'] = {
        'kinematics_solver': 'stretch_kinematics_plugin/StretchKinematicsPlugin',
        'kinematics_solver_search_resolution': 0.005,
        "kinematics_solver_timeout": 5,
        "kinematics_solver_attempts": 30,
    }
    moveit_config_dict['robot_description_kinematics']['stretch_arm'] = {
        'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution': 0.005,
        "kinematics_solver_timeout": 5,
        "kinematics_solver_attempts": 30,
    }

    moveit_config_dict['allow_trajectory_execution'] = True
    moveit_config_dict['max_safe_path_cost'] = 1
    moveit_config_dict['jiggle_fraction'] = 0.05
    moveit_config_dict['publish_planning_scene'] = True
    moveit_config_dict['publish_geometry_updates'] = True
    moveit_config_dict['publish_state_updates'] = True
    moveit_config_dict['publish_transforms_updates'] = True

    moveit = MoveItPy(node_name='moveit_py_node', config_dict=moveit_config_dict)

    moveit_plan = moveit.get_planning_component(planning_group)
    moveit_plan.set_workspace(-2.0, -2.0, -1.0, 2.0, 2.0, 2.0)

    planning_params = PlanRequestParameters(moveit)
    planning_params.planning_pipeline = 'ompl'
    planning_params.planner_id = 'RRTConnectkConfigDefault'
    planning_params.planning_attempts = 10
    planning_params.planning_time = 5.0
    planning_params.max_velocity_scaling_factor = 1.0
    planning_params.max_acceleration_scaling_factor = 1.0

    return moveit, moveit_plan, planning_params



class TrajectoryProcessor:
    def __init__(self):
        self.BASE_TRANS = 'translate_mobile_base'
        self.BASE_ROT = 'rotate_mobile_base'
        
        self.TOLERANCE = 1e-3  
        self.EPSILON = 1e-4  

    def process_trajectory(self, moveit_plan_result, current_joint_state):
        robot_traj = moveit_plan_result.trajectory.get_robot_trajectory_msg()
        
        # 1. Get current base positions
        start_trans = self._get_joint_val(current_joint_state, self.BASE_TRANS)
        
        # Note: We don't strictly need start_rot for Deltas, but we calculate yaw changes from it.
        # We ensure start_trans is not 0.0 to avoid driver bugs.
        if abs(start_trans) < self.EPSILON: start_trans = self.EPSILON

        # 2. Merge with STRICT NOISE FILTERING and DELTA ROTATION calculation
        full_traj = self._merge_moveit_output(
            robot_traj.joint_trajectory, 
            robot_traj.multi_dof_joint_trajectory,
            start_trans
        )
        
        # 3. Split segments (Updated for Delta Logic)
        trajectory_segments = self._split_and_filter(full_traj)
        
        return trajectory_segments

    def _get_joint_val(self, state, name):
        try:
            return state.position[state.name.index(name)]
        except (ValueError, AttributeError):
            return 0.0

    def _merge_moveit_output(self, arm_traj, base_multi_dof, start_trans):
        full_traj = JointTrajectory()
        full_traj.header = arm_traj.header
        full_traj.joint_names = list(arm_traj.joint_names) + [self.BASE_TRANS, self.BASE_ROT]
        
        # Translation remains Absolute (Accumulated)
        total_translate = start_trans
        
        prev_x, prev_y, prev_yaw = 0.0, 0.0, 0.0
        
        if len(base_multi_dof.points) > 0:
            start_t = base_multi_dof.points[0].transforms[0]
            prev_x = start_t.translation.x
            prev_y = start_t.translation.y
            prev_yaw = self._get_yaw(start_t.rotation)

        n_points = min(len(arm_traj.points), len(base_multi_dof.points))
        
        for i in range(n_points):
            arm_pt = arm_traj.points[i]
            base_pt = base_multi_dof.points[i]
            
            curr_t = base_pt.transforms[0]
            curr_x = curr_t.translation.x
            curr_y = curr_t.translation.y
            curr_yaw = self._get_yaw(curr_t.rotation)
            
            # Calculate Deltas
            dx = curr_x - prev_x
            dy = curr_y - prev_y
            # dyaw is the Delta Rotation
            dyaw = (curr_yaw - prev_yaw + np.pi) % (2 * np.pi) - np.pi
            
            raw_dist = np.sqrt(dx**2 + dy**2)
            raw_yaw = abs(dyaw)
            
            # --- STRICT DOMINANT AXIS FILTER ---
            # Determine if we are rotating or translating
            is_rot = raw_yaw > 1e-3
            is_trans = raw_dist > 1e-3
            
            if is_rot and is_trans:
                if raw_yaw > (2.0 * raw_dist): 
                    dx, dy = 0.0, 0.0 # Suppress translation
                else:
                    dyaw = 0.0 # Suppress rotation
            elif is_rot:
                dx, dy = 0.0, 0.0
            elif is_trans:
                dyaw = 0.0
            
            # -----------------------------------

            dist = np.sqrt(dx**2 + dy**2)
            if (dx * np.cos(prev_yaw) + dy * np.sin(prev_yaw)) < 0:
                dist = -dist
                
            total_translate += dist
            
            # Prepare commands
            # Translation is Accumulated (Absolute)
            if abs(total_translate) < self.EPSILON: cmd_trans = self.EPSILON
            else: cmd_trans = total_translate
            
            # Rotation is Delta (Relative)
            # Note: We must send EPSILON instead of 0.0 if not moving, 
            # otherwise the driver might crash with NoneType error.
            if abs(dyaw) < 1e-6: cmd_rot = self.EPSILON
            else: cmd_rot = dyaw
            
            new_pt = JointTrajectoryPoint()
            new_pt.time_from_start = arm_pt.time_from_start
            
            # Store [Absolute_Trans, Delta_Rot]
            new_pt.positions = list(arm_pt.positions) + [cmd_trans, cmd_rot]
            
            full_traj.points.append(new_pt)
            prev_x, prev_y, prev_yaw = curr_x, curr_y, curr_yaw
            
        return full_traj

    def _split_and_filter(self, full_traj):
        if not full_traj.points:
            return []

        segments = []
        idx_trans = full_traj.joint_names.index(self.BASE_TRANS)
        idx_rot = full_traj.joint_names.index(self.BASE_ROT)
        
        current_points = [full_traj.points[0]]
        current_mode = 'NONE' 

        for i in range(1, len(full_traj.points)):
            pt_prev = full_traj.points[i-1]
            pt_curr = full_traj.points[i]
            
            # Logic Update:
            # Translation is Absolute: Check diff (curr - prev)
            d_trans = pt_curr.positions[idx_trans] - pt_prev.positions[idx_trans]
            
            # Rotation is Delta: Check value (curr)
            # If the delta is larger than tolerance, we are rotating
            rot_val = pt_curr.positions[idx_rot]
            
            step_type = 'NONE'
            if abs(rot_val) > self.TOLERANCE:
                step_type = 'ROT'
            if abs(d_trans) > self.TOLERANCE:
                if step_type == 'ROT': step_type = 'BOTH'
                else: step_type = 'TRANS'
            
            if step_type == 'BOTH':
                # Split Logic
                if len(current_points) > 1:
                    segments.append(self._create_msg(full_traj.joint_names, current_points, current_mode))
                
                t_prev = self._to_sec(pt_prev.time_from_start)
                t_curr = self._to_sec(pt_curr.time_from_start)
                t_mid = t_prev + (t_curr - t_prev) / 2.0
                
                pt_mid = copy.deepcopy(pt_curr)
                self._set_sec(pt_mid.time_from_start, t_mid)
                
                # Create Midpoint
                mid_pos = list(pt_mid.positions)
                
                # For midpoint:
                # 1. Hold Translation at Prev
                mid_pos[idx_trans] = pt_prev.positions[idx_trans] 
                
                # 2. Rotation is Delta. 
                # If we split Rot->Trans:
                # Part 1 (Rot): Delta stays as is (Rotate).
                # Part 2 (Trans): Delta becomes 0 (Stop Rotating).
                # So pt_mid keeps the Rotation Delta. 
                # But pt_curr (which becomes start of Trans segment) needs 0 rotation.
                
                for j in range(len(full_traj.joint_names)):
                    if j not in [idx_trans, idx_rot]:
                        mid_pos[j] = (pt_prev.positions[j] + pt_curr.positions[j]) / 2.0
                pt_mid.positions = mid_pos
                
                segments.append(self._create_msg(full_traj.joint_names, [pt_prev, pt_mid], 'ROT'))
                
                # For the second segment (TRANS), we need the rotation delta to be effectively 0
                # (or EPSILON) because we are no longer rotating.
                pt_curr_trans = copy.deepcopy(pt_curr)
                pos_list = list(pt_curr_trans.positions)
                pos_list[idx_rot] = self.EPSILON
                pt_curr_trans.positions = pos_list
                
                segments.append(self._create_msg(full_traj.joint_names, [pt_mid, pt_curr_trans], 'TRANS'))
                
                current_points = [pt_curr_trans]
                current_mode = 'NONE'

            elif step_type != 'NONE' and step_type != current_mode and current_mode != 'NONE':
                segments.append(self._create_msg(full_traj.joint_names, current_points, current_mode))
                current_points = [pt_prev, pt_curr]
                current_mode = step_type
            
            else:
                current_points.append(pt_curr)
                if current_mode == 'NONE' and step_type != 'NONE':
                    current_mode = step_type

        if len(current_points) > 1:
            segments.append(self._create_msg(full_traj.joint_names, current_points, current_mode))
            
        return segments

    def _create_msg(self, full_names, points, mode):
        traj = JointTrajectory()
        
        indices_to_keep = []
        for idx, name in enumerate(full_names):
            if name == self.BASE_TRANS:
                if mode == 'TRANS' or mode == 'NONE': indices_to_keep.append(idx)
            elif name == self.BASE_ROT:
                if mode == 'ROT': indices_to_keep.append(idx)
            else:
                indices_to_keep.append(idx)
        
        traj.joint_names = [full_names[i] for i in indices_to_keep]
        start_t = self._to_sec(points[0].time_from_start)
        
        for pt in points:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = [pt.positions[i] for i in indices_to_keep]
            rel_t = self._to_sec(pt.time_from_start) - start_t
            self._set_sec(new_pt.time_from_start, rel_t)
            traj.points.append(new_pt)
            
        return traj

    def _get_yaw(self, q):
        roll, pitch, yaw = tf.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def _to_sec(self, dur):
        return dur.sec + dur.nanosec * 1e-9

    def _set_sec(self, dur_obj, seconds):
        dur_obj.sec = int(seconds)
        dur_obj.nanosec = int((seconds - int(seconds)) * 1e9)

