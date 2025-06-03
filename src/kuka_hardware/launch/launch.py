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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(f"kuka_{robot_family.perform(context)}_support"),
                    "urdf",
                    robot_model.perform(context) + ".urdf.xacro",
                ]
            ),
            " ",
            "use_fake_hardware:=",
            "true",
        ]
    )

    controller_config = (
        get_package_share_directory("kuka_resources")
        + f"/config/fake_hardware_config_6_axis.yaml"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_content, controller_config],
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description" : robot_description_content}],
    )

    nodes_to_start = [
        control_node,
        joint_state_spawner,
        controller_spawner,
        robot_state_publisher
    ]

    return nodes_to_start

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument("robot_model", default_value="kr6_r900_sixx"))
    launch_args.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
