# launch_new_nodes.py
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
            executable='video_subscriber_v2',
            name='video_subscriber_v2'
        ),
        Node(
            package='my_robot_package',
            executable='servo_control_v2',
            name='servo_control_v2'
        )
    ])