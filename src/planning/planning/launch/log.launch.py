from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='planning',
            executable='play_bag_node',
            name='play_bag_node',
            namespace='drone0',
            output='screen',
            remappings=[
                ('~gridmap', 'gridmap_inflate'),
                ('~replanState', 'replanState')
            ]
        )
    ])
