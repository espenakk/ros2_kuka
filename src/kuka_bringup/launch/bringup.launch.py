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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(f"kuka_{robot_family.perform(context)}_support"),
                    "urdf",
                    f"{robot_model.perform(context)}.urdf.xacro",
                ]
            ),
            " ",
            "use_fake_hardware:=",
            simulation.perform(context),
        ]
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
            "robot_description": robot_description_content
        }.items()
    )

    robot_description_kinematics = {
        "robot_description_kinematics": {
            "manipulator": {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin"}
        }
    }
    rviz_config_file = (
        os.path.join(get_package_share_directory("kuka_bringup"), "config", "config.rviz")
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_launch",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description_kinematics,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description" : robot_description_content},
                    {"use_sim_time": simulation}],
    )

    prediction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("kuka_control"), "launch", "ball_trajectory_prediction.launch.py")
        ),
        launch_arguments={
            "test_mode": simulation,
        }.items()
    )

    nodes_to_start = [
        moveit_launch,
        hardware_launch,
        robot_state_publisher,
        rviz_node,
        prediction_launch
    ]
    return nodes_to_start

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument("robot_model", default_value="kr6_r900_sixx"))
    launch_args.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    launch_args.append(DeclareLaunchArgument("simulation", default_value="true"))
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
