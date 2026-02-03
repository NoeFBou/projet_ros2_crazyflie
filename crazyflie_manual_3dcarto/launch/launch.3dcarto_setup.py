
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
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
