import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node

def generate_launch_description():

    pkg_navigation3d = get_package_share_directory('navigation3d')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_crazyflie_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')

    world_file = LaunchConfiguration('world_file')
    map_file = LaunchConfiguration('map_file')

    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_navigation3d, 'worlds', 'test27.sdf'),
        description='Chemin complet vers le fichier world SDF de Gazebo'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_navigation3d, 'maps', 'test27.bt'),
        description='Chemin complet vers le fichier de la carte (ex: .bt ou .pcd)'
    )

    model_folder = os.path.join(pkg_navigation3d, 'meshes', 'crazyflie')
    robot_sdf_path = os.path.join(model_folder, 'model.sdf')

    if not os.path.exists(robot_sdf_path):
        print(f"fichier SDF introuvable : {robot_sdf_path}")
    else:
        print(f"Fichier SDF trouve")

    params_file = os.path.join(pkg_navigation3d, 'config', 'planner.yaml')
    rviz_config_file = os.path.join(pkg_navigation3d, 'rviz', 'config5.rviz')
    nav3d_meshes_dir = os.path.join(pkg_navigation3d, 'meshes')

    resource_paths = [
        os.path.join(pkg_navigation3d, 'worlds'), ':',
        os.path.join(pkg_crazyflie_bringup, 'models'), ':',
        nav3d_meshes_dir
    ]

    set_env_vars = [
        SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_paths),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_paths),
    ]

    with open(robot_sdf_path, 'r') as infp:
        robot_desc = infp.read()
        
    common_params = [
        params_file,
        {'use_sim_time': True}
    ]
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }],
        arguments=[robot_sdf_path]
    )
    
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        # Utilisation de la LaunchConfiguration dans une liste pour la concaténation
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_crazyflie_bringup, 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],
        output='screen'
    )

    control = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.5},
            {'robot_prefix': 'crazyflie'},
            {'incoming_twist_topic': '/cmd_vel'},
            {'max_ang_z_rate': 0.4},
        ]
    )

    simple_mapper = Node(
        package='crazyflie_ros2_multiranger_simple_mapper',
        executable='simple_mapper_multiranger',
        name='simple_mapper',
        output='screen',
        parameters=[
            {'robot_prefix': 'crazyflie'},
            {'use_sim_time': True}
        ]
    )

    planner = Node(
        package="navigation3d",
        executable="planner",
        name="planner",
        output="screen",
        parameters=common_params,
    )
    
    marker_pose = Node(
        package="navigation3d",
        executable="interactive_marker_pose_stamped",
        name="interactive_marker_pose_stamped",
        output="screen",
        parameters=common_params,
    )

    traj_follower = Node(
        package="navigation3d",
        executable="trajectories_follower",
        name="trajectories_follower",
        output="screen",
        parameters=common_params,
    )
    
    supervisor = Node(
        package="navigation3d",
        executable="supervisor",
        name="supervisor",
        output="screen",
        parameters=common_params,
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'crazyflie',
            '-file', robot_sdf_path,
            '-x', '-2.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )
    
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'octomap_path': map_file, # Remplacé par le paramètre dynamique
            'height_map': True
        }]
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_map_file_cmd)
    
    for var in set_env_vars:
        ld.add_action(var)
        
    ld.add_action(octomap_server)
    #ld.add_action(robot_state_publisher)  # Ce truc marche pas :(
    ld.add_action(rviz2)
    ld.add_action(gazebo_sim)
    ld.add_action(bridge)
    ld.add_action(control)
    ld.add_action(simple_mapper)
    ld.add_action(spawn_robot)
    ld.add_action(planner)
    ld.add_action(marker_pose)
    ld.add_action(traj_follower)
    ld.add_action(supervisor)
    
    return ld