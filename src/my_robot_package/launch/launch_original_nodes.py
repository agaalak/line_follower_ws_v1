# launch_original_nodes.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='video_publisher',
            name='video_publisher'
        ),
        Node(
            package='my_robot_package',
            executable='video_subscriber',
            name='video_subscriber'
        ),
        Node(
            package='my_robot_package',
            executable='servo_control',
            name='servo_control'
        )
    ])