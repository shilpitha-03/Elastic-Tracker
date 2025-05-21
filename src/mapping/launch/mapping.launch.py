#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # path to your existing flat YAML
    mapping_share = get_package_share_directory('mapping')
    camera_yaml_path = PathJoinSubstitution([
        mapping_share, 'config', 'camera.yaml'
    ])

    # overrideable parameters
    down_sample_factor     = LaunchConfiguration('down_sample_factor',     default='2')
    depth_filter_tolerance = LaunchConfiguration('depth_filter_tolerance', default='0.15')
    depth_filter_mindist   = LaunchConfiguration('depth_filter_mindist',   default='0.2')
    depth_filter_margin    = LaunchConfiguration('depth_filter_margin',    default='2')
    p_min  = LaunchConfiguration('p_min',  default='-199')
    p_max  = LaunchConfiguration('p_max',  default='220')
    p_hit  = LaunchConfiguration('p_hit',  default='62')
    p_mis  = LaunchConfiguration('p_mis',  default='62')
    p_occ  = LaunchConfiguration('p_occ',  default='139')
    p_def  = LaunchConfiguration('p_def',  default='-199')
    resolution    = LaunchConfiguration('resolution',   default='0.15')
    local_x       = LaunchConfiguration('local_x',      default='20')
    local_y       = LaunchConfiguration('local_y',      default='20')
    local_z       = LaunchConfiguration('local_z',      default='5')
    inflate_size  = LaunchConfiguration('inflate_size', default='1')

    # Wrap the flat YAML under ros__parameters
    parameters = [
      {'ros__parameters': camera_yaml_path},
      {'down_sample_factor':     down_sample_factor},
      {'depth_filter_tolerance': depth_filter_tolerance},
      {'depth_filter_mindist':   depth_filter_mindist},
      {'depth_filter_margin':    depth_filter_margin},
      {'p_min':  p_min},
      {'p_max':  p_max},
      {'p_hit':  p_hit},
      {'p_mis':  p_mis},
      {'p_occ':  p_occ},
      {'p_def':  p_def},
      {'resolution':   resolution},
      {'local_x':      local_x},
      {'local_y':      local_y},
      {'local_z':      local_z},
      {'inflate_size': inflate_size},
    ]

    # component container to run the mapping nodelet
    container = ComposableNodeContainer(
        name='mapping_manager',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        # pass our wrapped parameters here
        parameters=parameters,
        composable_node_descriptions=[
            ComposableNode(
                package='mapping',
                plugin='mapping::MappingNodelet',
                name='mapping_nodelet',
                remappings=[
                    ('depth', 'depth'),
                    ('odom', 'odom'),
                    ('gridmap', 'gridmap'),
                    ('gridmap_inflate', 'gridmap_inflate'),
                ],
            )
        ]
    )

    ld = LaunchDescription()
    # Declare them so you can override via --ros-args -p ...
    for name, default in [
        ('down_sample_factor','2'),
        ('depth_filter_tolerance','0.15'),
        ('depth_filter_mindist','0.2'),
        ('depth_filter_margin','2'),
        ('p_min','-199'),
        ('p_max','220'),
        ('p_hit','62'),
        ('p_mis','62'),
        ('p_occ','139'),
        ('p_def','-199'),
        ('resolution','0.15'),
        ('local_x','20'),
        ('local_y','20'),
        ('local_z','5'),
        ('inflate_size','1'),
    ]:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    ld.add_action(container)
    return ld
