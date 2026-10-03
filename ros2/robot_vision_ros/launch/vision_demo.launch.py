from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_vision_ros',
            executable='dynamic_transform_demo',
            output='screen',
        ),

        Node(
            package='robot_vision_ros',
            executable='target_listener',
            output='screen',
            parameters=[{
                'tf_timeout_seconds': 0.1,
            }],
        ),

        Node(
            package='robot_vision_ros',
            executable='target_publisher',
            output='screen',
        ),
    ])