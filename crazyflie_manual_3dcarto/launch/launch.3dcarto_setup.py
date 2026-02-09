
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='octomap_server',
			executable='octomap_server_node',
			name='octomap_server',
			output='screen',
			parameters=[{
				'resolution': 0.05,
				'frame_id': 'map',
			}],
			remappings=[('cloud_in', '/my_3d_cloud')]
		),

		Node(
			package='crazyflie_manual_3dcarto',
			executable='scan_to_pcd',
			name='scan_to_pcd',
			output='screen',
			parameters=[],
		),
		Node(
			package='crazyflie_manual_3dcarto',
			executable='save_cloud',
			name='save_cloud',
			output='screen',
			parameters=[],
		),
	])
