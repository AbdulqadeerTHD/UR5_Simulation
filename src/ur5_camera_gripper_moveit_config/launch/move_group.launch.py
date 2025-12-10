#!/usr/bin/env python3
"""
Simple MoveIt move_group launch file - works without MoveItConfigsBuilder
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    moveit_config_pkg = get_package_share_directory('ur5_camera_gripper_moveit_config')
    ur_yt_sim_pkg = get_package_share_directory('ur_yt_sim')
    
    # File paths
    urdf_file = os.path.join(ur_yt_sim_pkg, 'urdf', 'ur5_camera_gripper_moveit.urdf.xacro')
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'ur.srdf')
    kinematics_file = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')
    moveit_controllers_file = os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml')
    ompl_planning_file = os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    # Generate robot description using xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        urdf_file,
        ' ur_type:=ur5',
        ' sim_gazebo:=false'
    ])
    
    # Read SRDF file content
    with open(srdf_file, 'r') as f:
        srdf_content = f.read()
    
    # MoveGroup node - load kinematics from file
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_content,
                'robot_description_semantic': srdf_content,
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'response_adapters': 'default_planner_response_adapters/AddTimeOptimalParameterization default_planner_response_adapters/ValidateSolution',
                'use_sim_time': use_sim_time,
            },
            kinematics_file,  # Load kinematics as params file
            joint_limits_file,
            moveit_controllers_file,
            ompl_planning_file,
        ],
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(launch_rviz),
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }
        ],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        launch_rviz_arg,
        move_group_node,
        rviz_node,
    ])

