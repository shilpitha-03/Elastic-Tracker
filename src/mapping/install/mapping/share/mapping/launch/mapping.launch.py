#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # paths
    mapping_share = get_package_share_directory('mapping')
    uav_sim_share = get_package_share_directory('uav_simulator')

    camera_yaml = PathJoinSubstitution([mapping_share, 'config', 'camera.yaml'])
    uav_launch = PathJoinSubstitution([uav_sim_share, 'launch', 'uav_simulator_launch.py'])

    # launch args (if you want to override any)
    down_sample_factor = LaunchConfiguration('down_sample_factor', default='2')
    depth_filter_tolerance = LaunchConfiguration('depth_filter_tolerance', default='0.15')
    depth_filter_mindist = LaunchConfiguration('depth_filter_mindist', default='0.2')
    depth_filter_margin = LaunchConfiguration('depth_filter_margin', default='2')
    p_min = LaunchConfiguration('p_min', default='-199')
    p_max = LaunchConfiguration('p_max', default='220')
    p_hit = LaunchConfiguration('p_hit', default='62')
    p_mis = LaunchConfiguration('p_mis', default='62')
    p_occ = LaunchConfiguration('p_occ', default='139')
    p_def = LaunchConfiguration('p_def', default='-199')
    resolution = LaunchConfiguration('resolution', default='0.15')
    local_x = LaunchConfiguration('local_x', default='20')
    local_y = LaunchConfiguration('local_y', default='20')
    local_z = LaunchConfiguration('local_z', default='5')
    inflate_size = LaunchConfiguration('inflate_size', default='1')

    container = ComposableNodeContainer(
        name='mapping_manager',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multithreaded
        output='screen',
        parameters=[camera_yaml],
        arguments=['--ros-args', '--log-level', 'info'],
        composable_node_descriptions=[
            ComposableNode(
                package='mapping',
                plugin='mapping::MappingNodelet',
                name='mapping_nodelet',
                remappings=[
                    ('odom', 'odom'),
                    ('depth', 'depth'),
                    ('gridmap', 'gridmap'),
                    ('gridmap_inflate', 'gridmap_inflate'),
                ],
                parameters=[{
                    'down_sample_factor': down_sample_factor,
                    'depth_filter_tolerance': depth_filter_tolerance,
                    'depth_filter_mindist': depth_filter_mindist,
                    'depth_filter_margin': depth_filter_margin,
                    'p_min': p_min,
                    'p_max': p_max,
                    'p_hit': p_hit,
                    'p_mis': p_mis,
                    'p_occ': p_occ,
                    'p_def': p_def,
                    'resolution': resolution,
                    'local_x': local_x,
                    'local_y': local_y,
                    'local_z': local_z,
                    'inflate_size': inflate_size,
                }],
            )
        ]
    )

    # Include the uav_simulator launch (assumes it's ported to Python)
    uav_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([uav_launch])
    )

    return LaunchDescription([
        # mapping parameters
        DeclareLaunchArgument('down_sample_factor', default_value='2'),
        DeclareLaunchArgument('depth_filter_tolerance', default_value='0.15'),
        DeclareLaunchArgument('depth_filter_mindist', default_value='0.2'),
        DeclareLaunchArgument('depth_filter_margin', default_value='2'),
        DeclareLaunchArgument('p_min', default_value='-199'),
        DeclareLaunchArgument('p_max', default_value='220'),
        DeclareLaunchArgument('p_hit', default_value='62'),
        DeclareLaunchArgument('p_mis', default_value='62'),
        DeclareLaunchArgument('p_occ', default_value='139'),
        DeclareLaunchArgument('p_def', default_value='-199'),
        DeclareLaunchArgument('resolution', default_value='0.15'),
        DeclareLaunchArgument('local_x', default_value='20'),
        DeclareLaunchArgument('local_y', default_value='20'),
        DeclareLaunchArgument('local_z', default_value='5'),
        DeclareLaunchArgument('inflate_size', default_value='1'),

        # component container + mapping nodelet
        container,

        # UAV simulator
        uav_sim,
    ])
