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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    wheel_type = LaunchConfiguration("wheel_type")
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    
    # Keepout zones
    keepout_mask_params_file = LaunchConfiguration('keepout_mask_params_file')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    keepout_lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']
    # Make re-written yaml
    keepout_mask_param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': keepout_mask_yaml_file}

    keepout_mask_configured_params = RewrittenYaml(
        source_file=keepout_mask_params_file,
        root_key=namespace,
        param_rewrites=keepout_mask_param_substitutions,
        convert_types=True)

    declare_keepout_mask_params_file_cmd = DeclareLaunchArgument(
            'keepout_mask_params_file',
            default_value='/nav2_config/keepout_params.yaml',
            description='Full path to the ROS2 parameters file to use')

    declare_keepout_mask_yaml_file_cmd = DeclareLaunchArgument(
            'keepout_mask',
            default_value='/maps/keepout_mask.yaml',
            description='Full path to filter mask yaml file to load')

    # Speed zones
    speed_params_file = LaunchConfiguration('speed_params_file')
    speed_mask_yaml_file = LaunchConfiguration('speed_mask')
    speed_lifecycle_nodes = ['speed_filter_mask_server', 'speed_costmap_filter_info_server']

    # Make re-written yaml
    speed_mask_param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': speed_mask_yaml_file}

    speed_mask_configured_params = RewrittenYaml(
        source_file=speed_params_file,
        root_key=namespace,
        param_rewrites=speed_mask_param_substitutions,
        convert_types=True)
    

    declare_speed_mask_params_file_cmd = DeclareLaunchArgument(
            'speed_params_file',
            default_value='/nav2_config/speed_params.yaml',
            description='Full path to the ROS2 parameters file to use')

    declare_speed_mask_yaml_file_cmd = DeclareLaunchArgument(
            'speed_mask',
            default_value='/maps/speed_mask.yaml',
            description='Full path to filter mask yaml file to load')

    # Preferred Lanes
    preferred_lanes_params_file = LaunchConfiguration('preferred_lanes_params_file')
    preferred_lanes_mask_yaml_file = LaunchConfiguration('preferred_lanes_mask')
    preferred_lanes_lifecycle_nodes = ['preferred_lanes_filter_mask_server', 'preferred_lanes_costmap_filter_info_server']

    ## Make re-written yaml
    preferred_lanes_mask_param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': preferred_lanes_mask_yaml_file}

    preferred_lanes_mask_configured_params = RewrittenYaml(
        source_file=preferred_lanes_params_file,
        root_key=namespace,
        param_rewrites=preferred_lanes_mask_param_substitutions,
        convert_types=True)
    

    declare_preferred_lanes_mask_params_file_cmd = DeclareLaunchArgument(
            'preferred_lanes_params_file',
            default_value='/nav2_config/preferred_lanes_params.yaml',
            description='Full path to the ROS2 parameters file to use')

    declare_preferred_lanes_mask_yaml_file_cmd = DeclareLaunchArgument(
            'preferred_lanes_mask',
            default_value='/maps/preferred_lanes_mask.yaml',
            description='Full path to filter mask yaml file to load')
    


    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')


    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value="WH01",
        description=(
            "Specify the type of wheel. If you select a value from the provided options ('WH01',"
            " 'WH02', 'WH04'), you can disregard the 'wheel_config_path' and"
            " 'controller_config_path' parameters. If you have custom wheels, set this parameter"
            " to 'CUSTOM' and provide the necessary configurations."
        ),
        choices=["WH01", "WH02", "WH04", "custom"],
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("panther_description"),
                "config",
                PythonExpression(["'", wheel_type, ".yaml'"]),
            ]
        ),
        description=(
            "Path to wheel configuration file. By default, it is located in "
            "'panther_description/config/<wheel_type arg>.yaml'. You can also specify the path "
            "to your custom wheel configuration file here. "
        ),
    )

    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("panther_controller"),
                "config",
                PythonExpression(["'", wheel_type, "_controller.yaml'"]),
            ]
        ),
        description=(
            "Path to controller configuration file. By default, it is located in"
            " 'panther_controller/config/<wheel_type arg>_controller.yaml'. You can also specify"
            " the path to your custom controller configuration file here. "
        ),
    )

    battery_config_path = LaunchConfiguration("battery_config_path")
    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("panther_gazebo"),
                "config",
                "battery_plugin_config.yaml",
            ]
        ),
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
    )

    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("panther_gazebo"),
                "config",
                "gz_bridge.yaml",
            ]
        ),
        description="Path to the parameter_bridge configuration file",
    )

    world_cfg = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=[
            "-r ",
            PathJoinSubstitution(
                [
                    get_package_share_directory("icon_sites_gz"),
                    "worlds",
                    "factory.sdf",
                ],
            ),
        ],
        description="SDF world file",
    )

    pose_x = LaunchConfiguration("pose_x")
    declare_pose_x_arg = DeclareLaunchArgument(
        "pose_x",
        default_value=["16.5"],
        # default_value=["1.5"],
        description="Initial robot position in the global 'x' axis.",
    )

    pose_y = LaunchConfiguration("pose_y")
    declare_pose_y_arg = DeclareLaunchArgument(
        "pose_y",
        # default_value=["13.0"],
        default_value=["20.0"],
        description="Initial robot position in the global 'y' axis.",
    )

    pose_z = LaunchConfiguration("pose_z")
    declare_pose_z_arg = DeclareLaunchArgument(
        "pose_z",
        default_value=["0.2"],
        description="Initial robot position in the global 'z' axis.",
    )

    rot_yaw = LaunchConfiguration("rot_yaw")
    declare_rot_yaw_arg = DeclareLaunchArgument(
        "rot_yaw", default_value=["0.0"], description="Initial robot orientation."
    )

    publish_robot_state = LaunchConfiguration("publish_robot_state")
    declare_publish_robot_state_arg = DeclareLaunchArgument(
        "publish_robot_state",
        default_value="True",
        description=(
            "Whether to launch the robot_state_publisher node."
            "When set to False, users should publish their own robot description."
        ),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={"gz_args": world_cfg}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "panther",
            "-allow_renaming",
            "true",
            "-topic",
            "robot_description",
            "-x",
            pose_x,
            "-y",
            pose_y,
            "-z",
            pose_z,
            "-Y",
            rot_yaw,
        ],
        output="screen",
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config_path}],
        output="screen",
    )



    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("panther_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "wheel_type": wheel_type,
            "wheel_config_path": wheel_config_path,
            "controller_config_path": controller_config_path,
            "battery_config_path": battery_config_path,
            "publish_robot_state": publish_robot_state,
            "use_sim": "True",
            "simulation_engine": "ignition-gazebo",
            "use_arm": "False",
            "use_ekf": "True",
        }.items(),
    )

    gps_wpf_dir = get_package_share_directory("nav2_gps")

    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [gps_wpf_dir,
                 'launch',
                 'ekf_navsat.launch.py'
                 ]
                )
        ),
        launch_arguments={
            "use_sim":"True",
        }.items(),
        condition=UnlessCondition("True")
    )

    lidar_sensor_suite_launch =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("lidar_sensor_suite_bringup"),
                    "launch",
                    "lidar_sensor_suite.launch.py",
                ]
            )
        )
    )   

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "bringup_launch.py"
                ]
            )
        ),
        launch_arguments={
            "params_file":"/nav2_config/nav2_params.yaml",
            "use_sim_time":"True",
            "map":"/maps/map.yaml"
        }.items()
    )

    start_keepout_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='keepout_lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': keepout_lifecycle_nodes}])
    
    start_keepout_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[keepout_mask_configured_params])

    start_keepout_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[keepout_mask_configured_params])
    
    start_speed_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='speed_lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': speed_lifecycle_nodes}])

    start_speed_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[speed_mask_configured_params])

    start_speed_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[speed_mask_configured_params])

    start_preferred_lanes_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='preferred_lanes_lifecycle_manager_costmap_filters',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': preferred_lanes_lifecycle_nodes}])
    
    start_preferred_lanes_map_server_cmd = Node(
            package='nav2_map_server',
            executable='map_server',
            name='preferred_lanes_filter_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[preferred_lanes_mask_configured_params])

    start_preferred_lanes_costmap_filter_info_server_cmd = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='preferred_lanes_costmap_filter_info_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[preferred_lanes_mask_configured_params])


    other_action_timer = TimerAction(
        period=20.0,
        actions=[
            lidar_sensor_suite_launch,
            # robot_localization_cmd,
            nav2_launch,
            # resize_window
            start_keepout_lifecycle_manager_cmd,
            start_keepout_map_server_cmd,
            start_keepout_costmap_filter_info_server_cmd,
            start_speed_lifecycle_manager_cmd,
            start_speed_map_server_cmd,
            start_speed_costmap_filter_info_server_cmd,
            start_preferred_lanes_lifecycle_manager_cmd,
            start_preferred_lanes_map_server_cmd,
            start_preferred_lanes_costmap_filter_info_server_cmd
        ],
    )

    # use_rviz = LaunchConfiguration('use_rviz')
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', rviz_config_file],
        output='screen')
    


    return LaunchDescription(
        [
            declare_world_arg,
            declare_pose_x_arg,
            declare_pose_y_arg,
            declare_pose_z_arg,
            declare_rot_yaw_arg,
            declare_wheel_type_arg,
            declare_wheel_config_path_arg,
            declare_controller_config_path_arg,
            declare_battery_config_path_arg,
            declare_gz_bridge_config_path_arg,
            declare_publish_robot_state_arg,
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_autostart_cmd,
            declare_keepout_mask_params_file_cmd,
            declare_keepout_mask_yaml_file_cmd,
            declare_speed_mask_params_file_cmd,
            declare_speed_mask_yaml_file_cmd,
            declare_preferred_lanes_mask_params_file_cmd,
            declare_preferred_lanes_mask_yaml_file_cmd,
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
            SetParameter(name="use_sim_time", value=True),
            gz_sim,
            gz_bridge,
            gz_spawn_entity,
            bringup_launch,
            start_rviz_cmd,
            other_action_timer,
        ]
    )