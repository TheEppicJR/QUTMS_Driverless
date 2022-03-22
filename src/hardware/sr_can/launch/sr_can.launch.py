"""
launch with `ros2 launch sr_can sr_can.launch.py`
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=[
            'Including launch file located at: ', ThisLaunchFileDir(), '/sr_can.launch.py'
        ]),
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource([ThisLaunchFileDir(), '/example.launch.py']),
        #     launch_arguments=['rosbridge_websocket_launch.xml'],
        # ),
        Node(
            package='rosboard',
            executable='rosboard_node',
        ),
        Node(
            package='sr_can',
            executable='sr_cam',
        ),
    ])