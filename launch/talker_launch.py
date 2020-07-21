from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
			Node(package = 'fim_track_2',
				node_namespace = 'talker',
				node_executable = 'talker',
				node_name = 'tk',
				arguments = ['ha','ha','hoho']
				)
		])