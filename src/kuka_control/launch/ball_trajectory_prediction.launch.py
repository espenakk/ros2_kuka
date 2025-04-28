from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    test_mode = LaunchConfiguration('test_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Enable test mode to use fake ball publisher'
        ),

        # Always start the ball trajectory predictor node
        Node(
            package='kuka_control',
            executable='ball_trajectory_prediction_node',
            name='ball_trajectory_prediction_node',
            output='screen'
        ),

        # Conditionally start the test ball publisher node
        Node(
            package='kuka_control',
            executable='test_ball_publisher_node',
            name='test_ball_publisher_node',
            output='screen',
            condition=IfCondition(test_mode)
        )
    ])
