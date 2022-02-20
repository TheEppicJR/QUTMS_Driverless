from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rrt_path_plan',
            namespace='rrt_plan',
            executable='rrt_plan',
            name='rrt_path'
        ),
        Node(
            package='sim_planning',
            namespace='pursuit',
            executable='pursuit',
            name='rrt_pursuit'
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            remappings=[
                ('/imu/data', '/imu'),
                ('/gps/fix', '/gps'),
            ],
            parameters=[
                ("magnetic_declination_radians","0"),
                ("yaw_offset", "0"),
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='robot_localization_ekf_node_odom',
            remappings=[
                ('/imu/data', '/imu'),
                ('/gps/fix', '/gps'),
            ],
            parameters=[
                ("frequency","10."),
                ("sensor_timeout", "0.2"),
                ("two_d_mode", "true"),
                ("publish_tf", "true"),
                ("map_frame", "map"),
                ("odom_frame", "odom"),
                ("base_link_frame", "base_link"),
                ("world_frame", "odom"),
                ("twist0", "turtle1/sensors/twist"),
                ("twist0_differential", "false"),
            ]
        ),
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='robot_localization_ekf_node_map',
            remappings=[
                ('/imu/data', '/imu'),
                ('/gps/fix', '/gps'),
            ],
            parameters=[
                ("frequency","10."),
                ("sensor_timeout", "0.2"),
                ("two_d_mode", "true"),
                ("publish_tf", "true"),
                ("map_frame", "map"),
                ("odom_frame", "odom"),
                ("base_link_frame", "base_link"),
                ("world_frame", "map"),
                ("twist0", "turtle1/sensors/twist"),
                ("pose0", "turtle1/sensors/pose"),
                ("odom1", "/your_state_estimation_node_topic"),
                ("odom1_differential", "false"),
            ]
        )
    ])