from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='false',
        description='Enable test mode to use fake ball publisher'
    )

    # Nodes
    ball_trajectory_prediction_node = Node(
        package='kuka_control',
        executable='ball_trajectory_prediction_node',
        name='ball_trajectory_prediction_node',
        output='screen'
    )

    test_ball_publisher_node = Node(
        package='kuka_control',
        executable='test_ball_publisher_node',
        name='test_ball_publisher_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('test_mode'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('kuka_control'),
            'config',
            'rviz_config.rviz'
        ])]
    )

    # Return Launch Description
    return LaunchDescription([
        test_mode_arg,
        ball_trajectory_prediction_node,
        test_ball_publisher_node,
        #rviz_node
    ])

