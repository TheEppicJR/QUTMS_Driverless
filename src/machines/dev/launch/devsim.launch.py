"""
launch with `ros2 launch dev devsim.launch.py`
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=[
            'Including launch file located at: ', ThisLaunchFileDir(), '/devsim.launch.py'
        ]),
        ExecuteProcess(cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml']),
        IncludeLaunchDescription( PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('state_estimation'), 'launch', 'state_estimation.launch.py'))),
        Node(
            package='rosboard',
            executable='rosboard_node',
        ),
        Node(
            package='sim_planning',
            executable='pursuit',
        ),
        Node(
            package='perception_debug',
            executable='perception_sim',
        ),
        Node(
            package='lidar_pipeline',
            executable='sim_simple',
        ),
        Node(
            package='vision_pipeline',
            executable='torch_detector',
        )
    ])