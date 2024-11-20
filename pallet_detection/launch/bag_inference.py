from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pallet_detection',
            executable='bag_subscriber',
            name='bag_subscriber',
            output='screen',
        ),
    ])