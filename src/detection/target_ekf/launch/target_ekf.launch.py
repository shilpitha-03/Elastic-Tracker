#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('target_ekf')
    camera_yaml = os.path.join(pkg_share, 'config', 'camera.yaml')

    return LaunchDescription([
        Node(
            package='target_ekf',
            executable='target_ekf_node',
            name='target_ekf',
            output='screen',
            parameters=[camera_yaml],     # load all keys in camera.yaml as parameters
            # remappings=[('/old_topic','/new_topic')],  # if needed
        ),
        # If you also want the sim node:
        Node(
            package='target_ekf',
            executable='target_ekf_sim_node',
            name='target_ekf_sim',
            output='screen',
            parameters=[camera_yaml],
        ),
    ])
