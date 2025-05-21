#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    mapping_share = get_package_share_directory('mapping')
    rs_share      = get_package_share_directory('realsense2_camera')

    # point at your D435 yaml
    camera_yaml = PathJoinSubstitution([mapping_share, 'config', 'D435.yaml'])

    # expose some args if you like
    use_mask           = LaunchConfiguration('use_mask', default='true')
    ekf_target_topic   = LaunchConfiguration('target', default='/target_ekf_node/target_odom')

    # 1) include the RealSense ROS2 launch (rs_launch.py)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([rs_share, 'launch', 'rs_launch.py'])
        ])
    )

    # 2) component container ("manager") under namespace drone0
    container = ComposableNodeContainer(
        name='manager',
        namespace='drone0',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='mapping',
                plugin='mapping::MappingNodelet',
                name='mapping',
                remappings=[
                    ('odom', '/ekf/ekf_odom'),
                    ('depth', '/camera/depth/image_rect_raw'),
                    ('gridmap',  'gridmap'),
                    ('gridmap_inflate','gridmap_inflate'),
                    ('target', ekf_target_topic),
                ],
                parameters=[
                  camera_yaml,
                  {
                    'down_sample_factor': 2,
                    'depth_filter_tolerance': 0.15,
                    'depth_filter_mindist': 0.2,
                    'depth_filter_margin': 2,
                    'p_min': -199,
                    'p_max': 220,
                    'p_hit': 62,
                    'p_mis': 62,
                    'p_occ': 139,
                    'p_def': -199,
                    'resolution': 0.15,
                    'local_x': 20,
                    'local_y': 20,
                    'local_z': 5,
                    'inflate_size': 1,
                    'use_mask': use_mask,
                  }
                ]
            )
        ]
    )

    return LaunchDescription([
        # allow overriding use_mask or target via CLI:
        DeclareLaunchArgument('use_mask', default_value='true'),
        DeclareLaunchArgument('target',    default_value='/target_ekf_node/target_odom'),

        realsense_launch,
        GroupAction([ container ])
    ])
