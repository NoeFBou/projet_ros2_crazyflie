from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='10.0',
        description='Rate at which to compute and publish ADE (Hz)'
    )
    
    history_duration_ms_arg = DeclareLaunchArgument(
        'history_duration_ms',
        default_value='10000',
        description='Decay duration of points used during the ADE computation'
    )
    
    interpolation_tolerance_arg = DeclareLaunchArgument(
        'interpolation_tolerance',
        default_value='0.1',
        description='Time tolerance for interpolation (seconds)'
    )
    
    drift_evaluator_node = Node(
        package='drift_evaluator',
        executable='drift_evaluator_node',
        name='drift_evaluator',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('update_rate'),
            'history_duration_ms': LaunchConfiguration('history_duration_ms'),
            'interpolation_tolerance': LaunchConfiguration('interpolation_tolerance'),
        }],
        remappings=[
            # Maybe later
        ]
    )
    
    return LaunchDescription([
        update_rate_arg,
        history_duration_ms_arg,
        interpolation_tolerance_arg,
        drift_evaluator_node,
    ])
