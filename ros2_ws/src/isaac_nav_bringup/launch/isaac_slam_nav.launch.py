from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params = LaunchConfiguration('nav2_params')
    slam_params = LaunchConfiguration('slam_params')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_dir = get_package_share_directory('isaac_nav_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        ),

        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(pkg_dir, 'config', 'slam_params.yaml')
        ),

        # Static transform: chassis_link -> lidar (LiDAR mounted on robot)
        # NOTE: world -> chassis_link must come from Isaac Sim (dynamic)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='chassis_to_lidar',
            arguments=['0', '0', '0.5', '0', '0', '0', 'chassis_link', 'lidar'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': slam_params,
                'autostart': 'false',  # Let lifecycle_manager handle activation
            }.items()
        ),

        # Nav2 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('isaac_nav_bringup'),
                    'launch',
                    'navigation_launch_no_dock.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
            }.items()
        ),
    ])