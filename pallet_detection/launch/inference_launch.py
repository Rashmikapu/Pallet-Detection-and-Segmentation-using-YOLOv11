from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pallet_detection',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
        ),
        Node(
            package='pallet_detection',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen',
        ),
    ])