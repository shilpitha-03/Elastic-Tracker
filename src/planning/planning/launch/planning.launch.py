from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rhosVisibility_', default_value='10000.0'),
        DeclareLaunchArgument('rhoTracking_', default_value='1000.0'),
        DeclareLaunchArgument('target_name_', default_value='/target_ekf_odom'),
        
        Node(
            package='planning',
            executable='planning_node',
            name='planning',
            output='screen',
            parameters=[{
                'plan_hz': 20,
                'vmax': 3.0,
                'amax': 6.0,
                'rhoT': 100.0,
                'rhoP': 10000.0,
                'rhoV': 1000.0,
                'rhoA': 1000.0,
                'rhoTracking': '1000.0',
                'rhosVisibility': '10000.0',
                'theta_clearance': 0.8,
                'clearance_d': 0.2,
                'tolerance_d': 0.3,
                'tracking_dist': 2.5,
                'debug': False
            }],
            remappings=[
                ('~odom', 'odom'),
                ('~gridmap_inflate', 'gridmap_inflate'),
                ('~heartbeat', 'heartbeat'),
                ('~trajectory', 'trajectory'),
                ('~replanState', 'replanState'),
                ('~target', '/target_ekf_odom'),
                ('~triger', '/triger')
            ]
        )
    ])
