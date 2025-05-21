from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mockamap',
            executable='mockamap_node',
            name='mockamap_node',
            output='screen',
            parameters=['config/mockamap.yaml'],
            remappings=[('/mock_map', '/global_map')]
        ),
        Node(
            package='planning',
            executable='planning_node',
            name='planning',
            output='screen',
            parameters=[{
                'plan_hz': 20,
                'K': 8,
                'vmax': 1.5,
                'amax': 2.5,
                'rhoT': 100.0,
                'rhoP': 10000.0,
                'rhoV': 1000.0,
                'rhoA': 1000.0,
                'clearance_d': 0.2,
                'tolerance_d': 0.5,
                'tracking_dist': 3.0,
                'debug': False,
                'fake': True
            }],
            remappings=[
                ('~odom', 'odom'),
                ('~gridmap_inflate', 'gridmap_inflate'),
                ('~heartbeat', 'heartbeat'),
                ('~trajectory', 'trajectory'),
                ('~replanState', 'replanState'),
                ('~triger', '/move_base_simple/goal')
            ]
        )
    ])
