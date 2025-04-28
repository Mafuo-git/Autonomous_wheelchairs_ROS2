from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qr_follower_opencv',
            executable='qr_follow_opencv',
            name='qr_follower_opencv',
            output='screen'
        ),
    ])