from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multirobot_map_merge',
            executable='map_merge',
            name='map_merge',
            namespace='map_merge',
            output='screen',
            parameters=[
                {'robot_map_topic': 'map'},
                {'robot_namespace': ''},
                {'merged_map_topic': 'map'},
                {'world_frame': 'map'},
                {'known_init_poses': True},
                {'merging_rate': 0.5},
                {'discovery_rate': 0.05},
                {'estimation_rate': 0.1},
                {'estimation_confidence': 1.0}
            ]
        )
    ])
