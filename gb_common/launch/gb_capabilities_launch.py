# Copyright 2021 Intelligent Robotics Lab
#
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory('gb_common')
    config_dir = os.path.join(bringup_dir, 'config')
    config_file = os.path.join(config_dir, 'world.yaml')

    attention_manager_cmd = Node(
        package='gb_attention',
        executable='attention_server',
        output='screen')

    nav2_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(
          get_package_share_directory('gb_navigation'),
          'launch', 'nav2_tiago_launch.py')))

    wm_cmd = Node(
        package='gb_world_model',
        executable='world_model_main',
        output='screen',
        parameters=[
          config_dir + '/world.yml'
        ])

    ld = LaunchDescription()

    ld.add_action(attention_manager_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(wm_cmd)

    return ld
