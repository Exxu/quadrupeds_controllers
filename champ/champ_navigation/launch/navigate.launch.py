# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def conditionally_include_nav2(context, *args, **kwargs):
    nav2_args = {
        'params_file': LaunchConfiguration("nav_file"),
        'use_sim_time': LaunchConfiguration("use_sim_time")
    }

    if LaunchConfiguration("use_map_server").perform(context) == 'true':
        nav2_args['map'] = LaunchConfiguration("map_path")
        print('Using map server with map: ', nav2_args['map'].perform(context))
    

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
                ])
            ),
            launch_arguments=nav2_args.items()
        )
    ]

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('champ_navigation'), 'rviz', 'navigation.rviz']
    )

    # if user provided map, start map_server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_path'), 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_map_server'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='map_path', 
            default_value='',
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='use_map_server', 
            default_value='false',
            description='Use map server if true'
        ),

        DeclareLaunchArgument(
            name='nav_file', 
            description='Navigation2 params file'
        ),

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz_nav', 
            default_value='false',
            description='Run rviz'
        ),

        # Conditionally include nav2_bringup
        OpaqueFunction(function=conditionally_include_nav2),

        LogInfo(msg=LaunchConfiguration('map_path')),
        LogInfo(msg=LaunchConfiguration('nav_file')),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz_nav")),
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
        ),

        # Run map server conditionally
        # map_server_node
    ])