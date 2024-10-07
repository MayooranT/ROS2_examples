import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix

def generate_launch_description():
    # model = LaunchConfiguration('model')
    model = 'waffle'
    urdf_file = os.path.join(get_package_share_directory('turtlebot3_description'),'urdf', 'turtlebot3_{}.urdf'.format(model))
    model_folder = 'turtlebot3_' + model
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
    current_path = '/home/mayooran/Documents/examples_ros2'
    target_path = '/src/voronoi/worlds/simple_env_1.world'
    world_path = current_path + target_path
    # gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_share_directory("robot_description"), "share"))

    model = LaunchConfiguration('model', default='burger')
    first_tb3 = LaunchConfiguration('first_tb3', default='tb3_0')
    second_tb3 = LaunchConfiguration('second_tb3', default='tb3_1')
    third_tb3 = LaunchConfiguration('third_tb3', default='tb3_2')
    
    # Positions for the three robots
    first_tb3_x_pos = LaunchConfiguration('first_tb3_x_pos', default='-3.2')
    first_tb3_y_pos = LaunchConfiguration('first_tb3_y_pos', default='-1.5')
    first_tb3_z_pos = LaunchConfiguration('first_tb3_z_pos', default='0.0')
    first_tb3_yaw = LaunchConfiguration('first_tb3_yaw', default='0.0')
    second_tb3_x_pos = LaunchConfiguration('second_tb3_x_pos', default='-3.2')
    second_tb3_y_pos = LaunchConfiguration('second_tb3_y_pos', default='0.0')
    second_tb3_z_pos = LaunchConfiguration('second_tb3_z_pos', default='0.0')
    second_tb3_yaw = LaunchConfiguration('second_tb3_yaw', default='0.0')
    third_tb3_x_pos = LaunchConfiguration('third_tb3_x_pos', default='-3.2')
    third_tb3_y_pos = LaunchConfiguration('third_tb3_y_pos', default='1.5')
    third_tb3_z_pos = LaunchConfiguration('third_tb3_z_pos', default='0.0')
    third_tb3_yaw = LaunchConfiguration('third_tb3_yaw', default='0.0')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # first turtlebot
    RSP_0 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=first_tb3,
            parameters=[{'publish_frequency': 50.0, 'tf_prefix': first_tb3}],
            arguments=[urdf_file]
        )
    
    Spawn_0 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            namespace=first_tb3,
            arguments=[
                '-entity', first_tb3,
                '-file', urdf_path,
                '-x', first_tb3_x_pos,
                '-y', first_tb3_y_pos,
                '-z', first_tb3_z_pos,
                '-Y', first_tb3_yaw
            ],
            output="screen"
        )

    nav2_0 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'namespace' : first_tb3
            }.items()
        )

    # second turtlebot
    RSP_1 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=second_tb3,
            parameters=[{'publish_frequency': 50.0, 'tf_prefix': second_tb3}],
            arguments=[urdf_file]
        )
    
    Spawn_1 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            namespace=second_tb3,
            arguments=[
                '-entity', second_tb3,
                '-file', urdf_file,
                '-x', second_tb3_x_pos,
                '-y', second_tb3_y_pos,
                '-z', second_tb3_z_pos,
                '-Y', second_tb3_yaw
            ]
        )

    gmapping_1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('voronoi'), 'launch', 'gmapping.launch.py')),
            launch_arguments={'robot_namespace': second_tb3}.items()
        )

    # third turtlebot
    RSP_2 = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=third_tb3,
            parameters=[{'publish_frequency': 50.0, 'tf_prefix': third_tb3}],
            arguments=[urdf_file]
        )
    
    Spawn_2 = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_urdf',
            namespace=third_tb3,
            arguments=[
                '-entity', third_tb3,
                '-file', urdf_file,
                '-x', third_tb3_x_pos,
                '-y', third_tb3_y_pos,
                '-z', third_tb3_z_pos,
                '-Y', third_tb3_yaw
            ]
        )

    gmapping_2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('voronoi'), 'launch', 'gmapping.launch.py')),
            launch_arguments={'robot_namespace': third_tb3}.items()
        )

    # Map merging
    map_merge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('voronoi'), 'launch', 'map_merge.launch.py'))
        )

    # RViz for visualization
    rviz =    Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(current_path,'src/voronoi', 'rviz', 'three_turtlebot3_slam.rviz')]
        )
    
    # Move base
    move_base_three = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('voronoi'), 'launch', 'move_base_three.launch.py')),
            launch_arguments={'model': model}.items()
        )

    slam_tool_box = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true'
            }.items()
        )
    
    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    
    ld.add_action(Spawn_0)
    # ld.add_action(Spawn_1)
    # ld.add_action(Spawn_2)
    ld.add_action(RSP_0)
    # ld.add_action(RSP_1)
    # ld.add_action(RSP_2)
    # ld.add_action(nav2_0)
    # ld.add_action(gmapping_1)
    # ld.add_action(gmapping_2)
    # ld.add_action(map_merge)
    # ld.add_action(move_base_three)
    # ld.add_action(rviz)
    # ld.add_action(slam_tool_box)

    return ld