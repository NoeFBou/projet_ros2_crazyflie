import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    pkg_localization = get_package_share_directory('crazyflie_localization')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_localization, 'worlds', 'test3.sdf')
    ply_path = os.path.join(home_dir, "crazyflie_mapping_demo/ros2_ws/src/crazyflie_localization/worlds/map_gazebo3.ply")
    rviz_config = os.path.join(pkg_localization, 'rviz', 'localization.rviz')

    # --- Configuration Environnement ---
    set_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(home_dir, 'crazyflie_mapping_demo/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models')
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(
                get_package_share_directory('ros_gz_crazyflie_bringup'),
                'config',
                'ros_gz_crazyflie_bridge.yaml'
            )
        }],
        output='screen'
    )

    localization_node = Node(
        package='crazyflie_localization',
        executable='localization',
        name='crazyflie_pf_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'ply_path': ply_path
        }]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        set_env,
        gazebo,
        bridge,
        localization_node,
        rviz2
    ])