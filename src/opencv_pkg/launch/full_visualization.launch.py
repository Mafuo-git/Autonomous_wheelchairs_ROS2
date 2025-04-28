from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
#ros2 run wheelchair-caninterface wheel_speed_to_odometry 

#ros2 launch ros2_socketcan socket_can_bridge.launch.xml
#ros2 run wheelchair-caninterface caninterface 
#ros2 launch opencv_pkg full_visualization.launch.py


def generate_launch_description():
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen'
    )

    qr_opencv_node = ExecuteProcess(
        cmd=['ros2', 'run', 'opencv_pkg', 'qr_opencv'],
        output='screen'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'enable_depth': 'false',
            'camera_namespace': '/'
        }.items()
    )

    odom = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(
    		os.path.join(
    		    get_package_share_directory('wheelchair-caninterface'),
    		    'launch',
    		    'odom.launch.py'
    		)
    	)
    )
    
    return LaunchDescription([
        plotjuggler_node,
        qr_opencv_node,
        realsense_launch,
        odom
    ])

