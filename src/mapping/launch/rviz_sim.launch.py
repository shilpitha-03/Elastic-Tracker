#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # locate the RVIZ config in the mapping package
    mapping_share = get_package_share_directory('mapping')
    rviz_config = os.path.join(mapping_share, 'config', 'rviz_sim.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
