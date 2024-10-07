from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create LaunchDescription object
    robot_namespace = LaunchConfiguration('robot_namespace')

    gmapping_node = Node(
        package='gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': [robot_namespace , '/map'],
            'odom_frame': [robot_namespace , '/odom'],
            'base_frame': [robot_namespace , '/base_footprint'],
            'map_update_interval': 1.0,
            'maxUrange': 4.0,
            'minimumScore': 50,
            'linearUpdate': 0.2,
            'angularUpdate': 0.2,
            'temporalUpdate': 0.5,
            'delta': 0.1,
            'lskip': 0,
            'particles': 30,
            'sigma': 0.05,
            'kernelSize': 1,
            'lstep': 0.05,
            'astep': 0.05,
            'iterations': 5,
            'lsigma': 0.075,
            'ogain': 3.0,
            'srr': 0.01,
            'srt': 0.02,
            'str': 0.01,
            'stt': 0.02,
            'resampleThreshold': 0.5,
            'xmin': -8.0,
            'ymin': -8.0,
            'xmax': 8.0,
            'ymax': 8.0,
            'llsamplerange': 0.01,
            'llsamplestep': 0.01,
            'lasamplerange': 0.005,
            'lasamplestep': 0.005
        }],
        remappings=[('scan', 'scan')]
    )


    ld = LaunchDescription()
    ld.add_action(gmapping_node)
    return ld
