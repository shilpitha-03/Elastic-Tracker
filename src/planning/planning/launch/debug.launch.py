from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planning',
            executable='planning_node',
            name='planning',
            namespace='drone0',
            output='screen',
            parameters=[{
                'plan_hz': 20,
                'K': 8,
                'vmax': 1.0,
                'amax': 2.0,
                'rhoT': 100.0,
                'rhoP': 10000.0,
                'rhoV': 1000.0,
                'rhoA': 1000.0,
                'rhoTracking': 1000.0,
                'rhosVisibility': 10000.0,
                'theta_clearance': 0.6,
                'clearance_d': 0.2,
                'tolerance_d': 0.3,
                'tracking_dist': 2.6,
                'tracking_dur': 3.0,
                'tracking_dt': 0.2,
                'debug': True,
                'fake': False
            }],
            remappings=[
                ('~odom', '/vins_estimator/imu_propagate'),
                ('~gridmap_inflate', 'gridmap_inflate'),
                ('~heartbeat', 'heartbeat'),
                ('~trajectory', 'trajectory'),
                ('~replanState', 'replanState'),
                ('~target', '/target_ekf_odom'),
                ('~triger', '/triger')
            ]
        )
    ])
