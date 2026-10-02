import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription, SomeSubstitutionsType, Substitution
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


class OffsetParser(Substitution):
    def __init__(
            self,
            number: SomeSubstitutionsType,
            offset: float,
    ) -> None:
        self.__number = number
        self.__offset = offset

    def perform(
            self,
            context: LaunchContext = None,
    ) -> str:
        number = float(self.__number.perform(context))
        return f'{number + self.__offset}'


ARGUMENTS = [
     DeclareLaunchArgument('sim_gui', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to show the simulator UI'),
   DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('slam', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to run slam'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to run slam synced'),
    DeclareLaunchArgument('localization', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to run localization'),
    DeclareLaunchArgument('nav2', default_value='true',
                          choices=['true', 'false'],
                          description='Run nav2'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='world_1',
                          description='simple_sim World'),
    DeclareLaunchArgument('model', default_value='standard',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model'),
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(get_package_share_directory('air_bringup'), 'config', 'nav2.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
]


def generate_launch_description():

    # Directories
    pkg_air_bringup = get_package_share_directory(
        'air_bringup')

    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_turtlebot4_navigation = get_package_share_directory(
        'turtlebot4_navigation')
    pkg_turtlebot4_viz = get_package_share_directory(
        'turtlebot4_viz')

    pkg_irobot_create_common_bringup = get_package_share_directory(
        'irobot_create_common_bringup')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')


    # Paths
    rviz_launch = PathJoinSubstitution(
        [pkg_turtlebot4_viz, 'launch', 'view_robot.launch.py'])
    nav_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav_bringup.launch.py'])
    #nav_launch = PathJoinSubstitution(
    #    [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
    create3_nodes_launch = PathJoinSubstitution(
        [pkg_air_bringup, 'create3_nodes.launch.py'])
    robot_description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py'])
    dock_description_launch = PathJoinSubstitution(
        [pkg_irobot_create_common_bringup, 'launch', 'dock_description.launch.py'])

    nav2_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'nav2.launch.py'])
    localization_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])
    slam_launch = PathJoinSubstitution(
        [pkg_turtlebot4_navigation, 'launch', 'slam.launch.py'])


    # Parameters
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution(
            [pkg_air_bringup, 'maps', 'office_1.yaml']),
        description='Full path to map yaml file to load')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')    
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_description_launch]),
        launch_arguments=[('model', LaunchConfiguration('model')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time'))]
    )

    # Dock description
    x_dock = OffsetParser(x, 0.157)
    yaw_dock = OffsetParser(yaw, 3.1416)
    dock_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dock_description_launch]),
        launch_arguments={'gazebo': 'ignition'}.items()
    )
    
    # simple_sim
    simple_sim = Node(
          package='air_simple_sim',
          executable='simple_sim',
          output='screen',
          parameters=[{'world': LaunchConfiguration('world'), 'platform_type':'turtlebot4', 'gui': LaunchConfiguration('sim_gui')}]
    )

        # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
            ('params_file', LaunchConfiguration('nav2_params_file'))            
        ],
        condition=IfCondition(LaunchConfiguration('nav2'))
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([localization_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
            ('map', LaunchConfiguration('map')),
            ('params', LaunchConfiguration('nav2_params_file'))                        
        ],
        condition=IfCondition(LaunchConfiguration('localization'))
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time),
            ('sync', LaunchConfiguration('sync')),
            ('params', LaunchConfiguration('nav2_params_file'))  # Failed to compute odom pose          
        ],
        condition=IfCondition(LaunchConfiguration('slam'))
    )


    # Rviz2
    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav_launch]),
        launch_arguments=[('slam', LaunchConfiguration('slam')),
                          ('nav2', LaunchConfiguration('nav2')),
                          ('localization', LaunchConfiguration('localization')),
                          ('use_sim_time', LaunchConfiguration('use_sim_time')),
                          ('map', LaunchConfiguration('map')),
                          ('params_file', LaunchConfiguration('nav2_params_file'))
                          ]
    )

    # Create3 nodes
    create3_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([create3_nodes_launch])
    )

    # RPLIDAR static transforms
    rplidar_stf = Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                'rplidar_link', [LaunchConfiguration('robot_name'), '/rplidar_link/rplidar']]
        )

    # OAKD static transforms
    oakd_pro_stf = Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'oakd_pro_rgb_camera_optical_frame',
                [LaunchConfiguration('robot_name'), '/oakd_pro_rgb_camera_frame/rgbd_camera']
            ],
            condition=LaunchConfigurationEquals('model', 'standard')
        )

    oakd_lite_stf = Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0',
                'oakd_lite_rgb_camera_optical_frame',
                [LaunchConfiguration('robot_name'), '/oakd_lite_rgb_camera_frame/rgbd_camera']
            ],
            condition=LaunchConfigurationEquals('model', 'lite')
        )

    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(simple_sim)
    #ld.add_action(rviz2)
    ld.add_action(robot_description)
    ld.add_action(dock_description)
    ld.add_action(create3_nodes)
    ld.add_action(nav2)    
    ld.add_action(localization)    
    ld.add_action(slam)    
    #### ld.add_action(navigation) # old
    ld.add_action(rplidar_stf)
    ld.add_action(oakd_pro_stf)
    ld.add_action(oakd_lite_stf)
    return ld
