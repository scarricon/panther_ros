#!/usr/bin/env python3

# Copyright 2023 Husarion sp. z o.o.
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
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from visualization_msgs.msg import Marker





def generate_launch_description():
    env_name = LaunchConfiguration("env_name")
    declare_env_name = DeclareLaunchArgument(
        "env_name",
        default_value="HouseOfPhoenix",
        description="Name of environment deployed",
    )

    load_mesh = Node(
        package="icon_sites_gz",
        executable="environment_mesh",
        parameters=[
            {'env_name':'HouseOfPhoenix'},
            {'x_pose':64.8},
            {'y_pose':81.75}
        ],
        output='both'
    )

    rosbag_filename = '/home/ssrmist/rosbag2_2024_02_02-09_50_36'
    rosplay_process = ExecuteProcess(
                    cmd=[
                        "ros2",
                        "bag",
                        "play",
                        rosbag_filename
                    ],
                    output="screen")

    # use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = '/home/ssrmist/.rviz2/default_playback.rviz'
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')
    


    return LaunchDescription(
        [
            declare_env_name,
            start_rviz_cmd,
            load_mesh,
            rosplay_process
        ]
    )

