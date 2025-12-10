from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import launch


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            description="Type/series of used UR robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_yt_sim",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur5_camera_gripper_moveit.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur5_camera_gripper_moveit_config",
            description="MoveIt config package.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Make MoveIt use simulation time.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments - get actual values from context
    ur_type_val = context.launch_configurations.get("ur_type", "ur5")
    description_package_val = context.launch_configurations.get("description_package", "ur_yt_sim")
    description_file_val = context.launch_configurations.get("description_file", "ur5_camera_gripper_moveit.urdf.xacro")
    moveit_config_package_val = context.launch_configurations.get("moveit_config_package", "ur5_camera_gripper_moveit_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Get package share directories
    from ament_index_python.packages import get_package_share_directory
    description_pkg_share = get_package_share_directory(description_package_val)
    moveit_config_pkg_share = get_package_share_directory(moveit_config_package_val)
    
    import os
    urdf_file = os.path.join(description_pkg_share, "urdf", description_file_val)
    srdf_file = os.path.join(moveit_config_pkg_share, "config", "ur.srdf")
    kinematics_file = os.path.join(moveit_config_pkg_share, "config", "kinematics.yaml")
    controllers_file = os.path.join(moveit_config_pkg_share, "config", "moveit_controllers.yaml")

    moveit_config = (
        MoveItConfigsBuilder(moveit_config_package_val)
        .robot_description(
            file_path=urdf_file,
            mappings={
                "ur_type": ur_type_val,
                "sim_gazebo": "false",
            },
        )
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .trajectory_execution(file_path=controllers_file)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz - use default config if custom one doesn't exist
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        run_move_group_node,
        rviz_node,
    ]

    return nodes_to_start

