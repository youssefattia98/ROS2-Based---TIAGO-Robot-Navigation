#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    optional_nodes = []
    package_dir = get_package_share_directory('sofar_tiago')
    world = LaunchConfiguration('world')
    my_map = LaunchConfiguration('map')
    mode = LaunchConfiguration('mode')
    nav_params = LaunchConfiguration('nav_params')
    slam_params_file = LaunchConfiguration('slam_params')
    slam_params_file_local = LaunchConfiguration('slam_params_local')
    slam_params_file_continue = LaunchConfiguration('slam_params_continue')
    use_rviz = LaunchConfiguration('rviz', default=False)
    use_nav = LaunchConfiguration('nav', default=False)
    use_slam = LaunchConfiguration('slam', default=False)
    use_localization_only = LaunchConfiguration('localization_only', default=False)
    use_continue = LaunchConfiguration('continue', default=False)
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_webots.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    nav2_map = PathJoinSubstitution([package_dir, 'resource', my_map])
    nav2_params = PathJoinSubstitution([package_dir, 'params', nav_params])
    slam_params = PathJoinSubstitution([package_dir, 'params', slam_params_file])
    slam_params_local = PathJoinSubstitution([package_dir, 'params', slam_params_file_local])
    slam_params_continue = PathJoinSubstitution([package_dir, 'params', slam_params_file_continue])
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    

    

    rviz_config = os.path.join(get_package_share_directory('sofar_tiago'), 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',nav2_params),
            ],
            condition=launch.conditions.IfCondition(use_nav)))
            
    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',nav2_params),
            ],
            condition=launch.conditions.IfCondition(use_localization_only)))
            
    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',nav2_params),
            ],
            condition=launch.conditions.IfCondition(use_slam)))
    
    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments=[
                ('map', nav2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',nav2_params),
            ],
            condition=launch.conditions.IfCondition(use_continue)))
            
    if 'slam_toolbox' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('slam_params_file',slam_params),
            ],
            condition=launch.conditions.IfCondition(use_slam)))
            
    if 'slam_toolbox' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('slam_params_file',slam_params_local),
            ],
            condition=launch.conditions.IfCondition(use_localization_only)))

    if 'slam_toolbox' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'lifelong_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('slam_params_file',slam_params_continue),
            ],
            condition=launch.conditions.IfCondition(use_continue)))

    return LaunchDescription([
        DeclareLaunchArgument(
            #can be replaced with default1.wbt or default2.wbt for another world
            'world',
            default_value='default.wbt',
            description='Choose one of the world files from `/sofar_tiago/worlds` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='map.yaml',
            description='Choose one of the map files from `/sofar_tiago/resource` directory'
        ),
        DeclareLaunchArgument(
            'nav_params',
            default_value='navigation_params.yaml',
            description='Choose one of the param files from `/sofar_tiago/params` directory'
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value='slam_params.yaml',
            description='Choose one of the param files from `/sofar_tiago/params` directory'
        ),
        DeclareLaunchArgument(
            'slam_params_local',
            default_value='slam_params_localization.yaml',
            description='Choose one of the param files from `/sofar_tiago/params` directory'
        ),
        DeclareLaunchArgument(
            'slam_params_continue',
            default_value = 'mapper_params_lifelong.yaml',
            description= 'Choose one of the param files from `sofar_tiago/params` directory'
        ),
        
        
        
        rviz
    ] + optional_nodes)
