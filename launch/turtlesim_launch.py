from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
			Node(package = 'turtlesim',
				node_namespace = 'ts1',
				node_executable = 'turtlesim_node',
				node_name = 'sim'
				)
		])