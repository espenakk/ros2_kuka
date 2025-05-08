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

    kuka_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kuka_agilus_support"), "launch", f"test_{robot_model.perform(context)}.launch.py")
        )
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kuka_moveit"), "launch", "moveit_planning.launch.py")
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_family": robot_family
        }.items()
    )


    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kuka_hardware"), "launch", "launch.py")
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_family": robot_family,
            "simulation": simulation
        }.items()
    )

    nodes_to_start = [
        moveit_launch,
        kuka_driver_launch,
        hardware_launch
    ]
    return nodes_to_start

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument("robot_model", default_value="kr6_r700_sixx"))
    launch_args.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    launch_args.append(DeclareLaunchArgument("simulation", default_value="true"))
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
