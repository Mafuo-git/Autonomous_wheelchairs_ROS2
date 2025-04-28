from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='wheelchair-caninterface',
			executable='wheel_speed_to_odometry',
			name='wheel_odometry_node',
			output='screen'
		)
	])
