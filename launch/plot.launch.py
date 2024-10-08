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
        'input_csv_path',
        default_value=pkg_dir + '/data/output.csv',
        description='Full path to the output CSV file')
    declare_bag_file_path_cmd = DeclareLaunchArgument(
        'output_dir',
        default_value=pkg_dir + '/data/',
        description='Full path to the bag file to analyze')
    declare_plot_uncertainties_cmd = DeclareLaunchArgument(
        'plot_uncertainties',
        default_value='false',
        description='Bool to plot uncertainties')
    declare_config_file_path_cmd = DeclareLaunchArgument(
        'config_file_path',
        default_value=pkg_dir + '/config/config.yaml',
        description='Config file path to register custom msgs')
    generate_csv_cmd = Node(
        package='plansys2_data_collector',
        executable='plot_generator',
        name='plot_generator',
        output='screen',
        parameters=[
            {'input_csv_path': LaunchConfiguration('input_csv_path')},
            {'output_dir': LaunchConfiguration('output_dir')},
            {'plot_uncertainties': LaunchConfiguration('plot_uncertainties')}
        ])
    
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_bag_file_path_cmd)
    ld.add_action(declare_csv_output_path_cmd)
    ld.add_action(declare_plot_uncertainties_cmd)

    ld.add_action(generate_csv_cmd)
    return ld