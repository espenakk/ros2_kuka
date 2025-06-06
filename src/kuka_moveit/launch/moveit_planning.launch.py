# Copyright 2022 Áron Svastits
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources.python_launch_description_source import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_family = LaunchConfiguration("robot_family")

    moveit_config_dir = get_package_share_directory("kuka_kr_moveit_config")

    curr_dir = get_package_share_directory("kuka_moveit")

    moveit_config = (
        MoveItConfigsBuilder("kuka_kr")
        .robot_description(
            file_path=get_package_share_directory(f"kuka_{robot_family.perform(context)}_support")
            + f"/urdf/{robot_model.perform(context)}.urdf.xacro"
        )
        .robot_description_semantic(
            f"{moveit_config_dir}/urdf/{robot_model.perform(context)}_arm.srdf"
        )
        .robot_description_kinematics(file_path=f"{curr_dir}/config/kinematics.yaml")
        .trajectory_execution(file_path=f"{moveit_config_dir}/config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(
            file_path=get_package_share_directory(f"kuka_{robot_family.perform(context)}_support")
            + f"/config/{robot_model.perform(context)}_joint_limits.yaml"
        )
        .to_moveit_configs()
    )

    move_group_server = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    point_to_legal_pose_node = Node(
        package="kuka_moveit",
        executable="point_to_legal_pose_node",
        output="screen",
    )

    trajectory_planning_node = Node(
        package="kuka_moveit",
        executable="trajectory_planning_node",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    plan_execution_node = Node(
        package="kuka_moveit",
        executable="plan_execution_node",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    to_start = [
        move_group_server,
        #point_to_legal_pose_node,
        trajectory_planning_node,
        plan_execution_node
    ]

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("robot_model", default_value="kr6_r900_sixx"))
    launch_arguments.append(DeclareLaunchArgument("robot_family", default_value="agilus"))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
