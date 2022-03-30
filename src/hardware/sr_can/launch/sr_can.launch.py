"""
launch with `ros2 launch sr_can sr_can.launch.py`
"""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_localization_dir = get_package_share_directory('state_estimation')
    parameters_file_path = os.path.join(robot_localization_dir, 'dual_ekf_navsat_sr.yaml')
    os.environ['FILE_PATH'] = str(robot_localization_dir)
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
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/local')]           
           ),
        Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/global')]
           ),           
        Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('imu/data', 'sr_imu/imu_acc_ar'),
                        ('gps/fix', 'daq/gps'), 
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')]           

           )  
    ])