
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
import xacro

def launch_setup(context, *args, **kwargs):
    camera_left_device = LaunchConfiguration("camera_left_device")
    camera_right_device = LaunchConfiguration("camera_right_device")

    camera_left_params = get_package_share_directory("camera") + "/config/params_left.yaml"
    camera_right_params = get_package_share_directory("camera") + "/config/params_right.yaml"
    detection_config_path = get_package_share_directory("camera") + "/config/config.toml"
    stereo_params_path = get_package_share_directory("camera") + "/config/stereo_params.yaml"

    camera_left_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera_left",
        parameters=[ParameterFile(camera_left_params),
                    {"video_device": camera_left_device.perform(context)}],
        namespace="camera_left"
    )

    camera_right_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera_right",
        parameters=[ParameterFile(camera_right_params),
                    {"video_device": camera_right_device.perform(context)}],
        namespace="camera_right"
    )

    detection_node = Node(
        package="camera",
        executable="detection_node",
        name="detection_node",
        parameters=[{'detection_config_path': detection_config_path},
                    {'stereo_params_path': stereo_params_path}],
    )

    nodes_to_start = [
        camera_left_node,
        camera_right_node,
        #detection_node,
    ]
    return nodes_to_start

def generate_launch_description():
    launch_args = []
    launch_args.append(DeclareLaunchArgument("camera_left_device", default_value="/dev/video0"))
    launch_args.append(DeclareLaunchArgument("camera_right_device", default_value="/dev/video2"))
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
