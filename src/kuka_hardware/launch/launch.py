import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")
    robot_description = LaunchConfiguration("robot_description")


    controller_config = (
        get_package_share_directory("kuka_resources")
        + f"/config/fake_hardware_config_6_axis.yaml"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ]
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller"
        ]
    )

    nodes_to_start = [
        control_node,
        joint_state_spawner,
        controller_spawner
    ]

    return nodes_to_start

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument("robot_model", default_value="kr6_r700_sixx"))
    launch_args.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    launch_args.append(DeclareLaunchArgument("robot_description", default_value="true"))
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
