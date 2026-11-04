"""Planning-only MoveIt. No controller is loaded and no DDS hardware publisher."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = Path(get_package_share_directory('d1_pick_place')) / 'config'
    model = {'robot_description': (cfg / 'd1.urdf').read_text(),
             'robot_description_semantic': (cfg / 'd1.srdf').read_text(),
             'robot_description_kinematics': {'arm': {
                 'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                 'kinematics_solver_search_resolution': .005,
                 'kinematics_solver_timeout': .1}},
             'robot_description_planning': yaml.safe_load((cfg / 'joint_limits.yaml').read_text())}
    ompl = {'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames',
                                 'default_planning_request_adapters/ValidateWorkspaceBounds',
                                 'default_planning_request_adapters/CheckStartStateBounds',
                                 'default_planning_request_adapters/CheckStartStateCollision'],
            'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization',
                                  'default_planning_response_adapters/ValidateSolution'],
            'planner_configs': {'RRTConnectkConfigDefault': {'type': 'geometric::RRTConnect', 'range': 0.0}},
            'arm': {'planner_configs': ['RRTConnectkConfigDefault'],
                    'longest_valid_segment_fraction': .005},
            'gripper': {'planner_configs': ['RRTConnectkConfigDefault']}}
    return LaunchDescription([
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[model, {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
                         'ompl': ompl, 'allow_trajectory_execution': False,
                         'publish_robot_description_semantic': True}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': model['robot_description']}]),
    ])
