"""
launch with `ros2 launch sr_can sr_can.launch.py`
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
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
        # DeclareLaunchArgument(

        # ),
        #ExecuteProcess(cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml']),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
        ),
        Node(
            package='rosboard',
            executable='rosboard_node',
        ),
        Node(
            package='sr_can',
            executable='sr_cam',
        ),
        Node(
            package='sr_can',
            executable='sr_can',
        ),
        Node(
            package='ros_openimu',
            executable='sr_imu',
        ),
        Node(
            package='sr_can',
            executable='sr_common',
        )
    ])