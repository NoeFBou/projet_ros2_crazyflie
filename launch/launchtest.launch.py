import os
import xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    AppendEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_pointcloud_builder = get_package_share_directory('pointcloud_builder')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_crazyflie_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')

    # path_to_models_source = os.path.join(
    #     os.environ.get('HOME'),
    #     'crazyflie_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie'
    # )
    # robot_sdf_path = os.path.join(path_to_models_source, 'crazyflie', 'model.sdf')

    models_parent_dir = os.path.join(
        os.environ.get('HOME'),
        'crazyflie_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models'
    )

    robot_sdf_path = os.path.join(models_parent_dir, 'crazyflie', 'model.sdf')

    if not os.path.exists(robot_sdf_path):
        print(f"fichier SDF introuvable : {robot_sdf_path}")
    else:
        print(f"Fichier SDF trouve")

    params = os.path.join(pkg_pointcloud_builder, 'config', 'planner.yaml')
    rviz_config_file = os.path.join(pkg_pointcloud_builder, 'rviz', 'config.rviz')
    world_path = os.path.join(pkg_pointcloud_builder, 'worlds', 'oui.sdf')
    resource_paths = [
        os.path.join(pkg_pointcloud_builder, 'worlds'), ':',
        os.path.join(pkg_crazyflie_bringup, 'models'), ':',
        models_parent_dir
    ]
    set_env_vars = [
            #SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='3.3'),
            #SetEnvironmentVariable(name='MESA_GLES_VERSION_OVERRIDE', value='3.3'),
            SetEnvironmentVariable(name='LIBGL_ALWAYS_SOFTWARE', value='1'),
            # SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=[
            #     os.path.join(pkg_pointcloud_builder, 'worlds'), ':',
            #     os.path.join(pkg_crazyflie_bringup, 'models'), ':',
            #     models_parent_dir
            # ]),
            SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=resource_paths),
            SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_paths),
        ]

    # gz_resource_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value=[
    #         os.path.join(pkg_pointcloud_builder, 'worlds'), ':',
    #         os.path.join(pkg_crazyflie_description, 'models'), ':',
    #         '/home/test/crazyflie_ws/src/crazyflie-simulation/simulator_files/gazebo'
    #     ]
    # )
    with open(robot_sdf_path, 'r') as infp:
        robot_desc = infp.read()

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
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
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
        package="pointcloud_builder",
        executable="planner",
        name="planner",
        output="screen",
        parameters=[params],
    )

    goal = Node(
        package="pointcloud_builder",
        executable="goal_manager",
        name="goal_manager",
        output="screen",
        parameters=[params],
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
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ],
        output='screen'
    )
    map_path = os.path.join(pkg_pointcloud_builder, 'maps', 'fr_campus.bt')
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'octomap_path': map_path,
            'height_map': True
        }]
    )

    ld = LaunchDescription()
    for var in set_env_vars:
        ld.add_action(var)
    ld.add_action(planner)
    ld.add_action(octomap_server)
    ld.add_action(robot_state_publisher)  #se truck marche pas :(
    ld.add_action(goal)
    ld.add_action(rviz2)
    ld.add_action(gazebo_sim)
    ld.add_action(bridge)
    ld.add_action(control)
    ld.add_action(simple_mapper)
    ld.add_action(spawn_robot)
    return ld