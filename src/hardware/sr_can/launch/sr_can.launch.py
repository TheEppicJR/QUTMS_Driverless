"""
launch with `ros2 launch state_estimation state_estimation.launch.py`
probably can tab to complete most of it
"""

# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # robot_localization_dir = get_package_share_directory('state_estimation')
    # parameters_file_path = os.path.join(robot_localization_dir, 'dual_ekf_navsat_example.yaml')
    # os.environ['FILE_PATH'] = str(robot_localization_dir)
    return LaunchDescription([
        launch_ros.actions.Node(
            package='sr_can', 
            executable='sr_can', 
            name='sr_can_driver',
            remappings=[]           
           ),
        launch_ros.actions.Node(
            package='sr', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
            remappings=[]
           )          
])