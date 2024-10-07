import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    return LaunchDescription([
        DeclareLaunchArgument('model', description='Model type'),
        DeclareLaunchArgument('robot_namespace', description='Namespace for the robot'),
        DeclareLaunchArgument('move_forward_only', default_value='false', description='Restrict movement to forward only'),

        GroupAction([
            # Use namespace for robot-specific nodes
            Node(
                package='move_base',
                executable='move_base',
                name='move_base',
                namespace=LaunchConfiguration('robot_namespace'),
                output='screen',
                parameters=[
                    # Load the default configuration for move_base from turtlebot3_navigation
                    {'base_local_planner': 'dwa_local_planner/DWAPlannerROS',
                     'global_costmap/scan/sensor_frame': LaunchConfiguration('robot_namespace') + '/base_scan',
                    'global_costmap/obstacle_layer/scan/sensor_frame': LaunchConfiguration('robot_namespace') + '/base_scan',
                    'global_costmap/global_frame': 'map',
                    'global_costmap/robot_base_frame': LaunchConfiguration('robot_namespace') + '/base_footprint',
                    'local_costmap/scan/sensor_frame': LaunchConfiguration('robot_namespace') + '/base_scan',
                    'local_costmap/obstacle_layer/scan/sensor_frame': LaunchConfiguration('robot_namespace') + '/base_scan',
                    'local_costmap/global_frame': LaunchConfiguration('robot_namespace') + '/odom',
                    'local_costmap/robot_base_frame': LaunchConfiguration('robot_namespace') + '/base_footprint',
                    # Apply conditional parameters for forward movement only
                    'DWAPlannerROS/min_vel_x': PythonExpression(['0.0' if LaunchConfiguration('move_forward_only') == 'true' else '-0.5'])
                     },
                    ParameterFile(
                        os.path.join(
                            get_package_share_directory('turtlebot3_navigation'),
                            'param',
                            'costmap_common_params_{}.yaml'.format(LaunchConfiguration('model'))
                        ),
                        allow_substs=True,
                        condition=IfCondition(PythonExpression(['not ', LaunchConfiguration('move_forward_only')]))
                    ),
                    ParameterFile(
                        os.path.join(
                            get_package_share_directory('turtlebot3_navigation'),
                            'param',
                            'local_costmap_params.yaml'
                        ),
                        allow_substs=True
                    ),
                    ParameterFile(
                        os.path.join(
                            get_package_share_directory('turtlebot3_navigation'),
                            'param',
                            'global_costmap_params.yaml'
                        ),
                        allow_substs=True
                    ),
                    ParameterFile(
                        os.path.join(
                            get_package_share_directory('turtlebot3_navigation'),
                            'param',
                            'move_base_params.yaml'
                        ),
                        allow_substs=True
                    ),
                    ParameterFile(
                        os.path.join(
                            get_package_share_directory('turtlebot3_navigation'),
                            'param',
                            'dwa_local_planner_params_{}.yaml'.format(LaunchConfiguration('model'))
                        ),
                        allow_substs=True
                    ) 
                ],
                remappings=[
                    # Remap for centralized map server (if needed, uncomment this section)
                    # ('tb3_0/map', '/map')
                ]
            )
        ])
    ])
