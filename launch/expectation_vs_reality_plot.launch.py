# Copyright 2024 Intelligent Robotics Lab
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('plansys2_data_collector')

    declare_csv_output_path_cmd = DeclareLaunchArgument(
        'main_path',
        default_value=pkg_dir + '/results/',
        description='Main path of the folder containing subfolders with the exported data')
    declare_config_file_path_cmd = DeclareLaunchArgument(
        'comparison_config_file_path',
        default_value=pkg_dir + '/config/comparison_config.yaml',
        description='Config file path to register custom msgs')

    comparison_plot_cmd = Node(
        package='plansys2_data_collector',
        executable='expectation_vs_reality_plot',
        name='expectation_vs_reality_plot',
        output='screen',
        parameters=[
            {'main_path': LaunchConfiguration('main_path')},
            LaunchConfiguration('comparison_config_file_path')
        ])
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_csv_output_path_cmd)
    ld.add_action(declare_config_file_path_cmd)
    ld.add_action(comparison_plot_cmd)

    return ld