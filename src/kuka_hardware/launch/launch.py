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
    simulation = LaunchConfiguration("simulation")
    client_ip = LaunchConfiguration("client_ip")
    client_port = LaunchConfiguration("client_port")

    startup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("kuka_rsi_driver"), "/launch/startup.launch.py"]
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_family": robot_family,
            "use_fake_hardware": simulation,
            "client_ip": client_ip,
            "client_port": client_port,
        }.items()
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
        startup_launch,
        joint_state_spawner,
        controller_spawner
    ]

    return nodes_to_start

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument("robot_model", default_value="kr6_r900_sixx"))
    launch_args.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    launch_args.append(DeclareLaunchArgument("simulation", default_value="true"))
    launch_args.append(DeclareLaunchArgument("client_port", default_value="59152"))
    launch_args.append(DeclareLaunchArgument("client_ip", default_value="0.0.0.0"))
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
