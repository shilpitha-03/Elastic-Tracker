from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mapping'), 'launch', 'mapping_launch.py')
            )
        ),
        Node(
            package='mapping',
            executable='mapping_vis_node',
            name='mapping_vis',
            output='screen',
            remappings=[
                ('~gridmap', 'gridmap'),
                ('~gridmap_inflate', 'gridmap_inflate')
            ],
            parameters=[{'robot_scale': 1.0}]
        ),
        Node(
            package='odom_visualization',
            executable='odom_visualization',
            name='odom_visualization',
            output='screen',
            parameters=[{'robot_scale': 1.0}],
            remappings=[
                ('~odom', 'odom')
            ]
        ),
        Node(
            package='target_ekf',
            executable='target_ekf_sim_node',
            name='target_ekf_sim_node',
            output='screen',
            parameters=[{
                'pitch_thr': 37,
                'check_fov': False
            }],
            remappings=[
                ('~yolo', '/target/odom'),
                ('~odom', 'odom'),
                ('~target_odom', '/target_ekf_odom')
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('planning'), 'launch', 'planning_launch.py')
            ),
            launch_arguments={
                'rhosVisibility_': '10000.0',
                'rhoTracking_': '1000.0',
                'target_name_': '/target_ekf_odom'
            }.items()
        )
    ])
