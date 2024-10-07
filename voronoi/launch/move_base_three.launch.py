import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the model argument
    model = LaunchConfiguration('model')

    # Get the path to the move_base_namespace launch file
    move_base_namespace_launch = os.path.join(
        get_package_share_directory('voronoi'),
        'launch',
        'move_base_namespace.launch.py'
    )

    return LaunchDescription([
        # Declare the 'model' argument
        DeclareLaunchArgument(
            'model',
            description='Model type for the TurtleBots'
        ),

        # Include for tb3_0
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_base_namespace_launch),
            launch_arguments={
                'robot_namespace': 'tb3_0',
                'model': model
            }.items()
        ),

        # Include for tb3_1
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_base_namespace_launch),
            launch_arguments={
                'robot_namespace': 'tb3_1',
                'model': model
            }.items()
        ),

        # Include for tb3_2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(move_base_namespace_launch),
            launch_arguments={
                'robot_namespace': 'tb3_2',
                'model': model
            }.items()
        ),
    ])
