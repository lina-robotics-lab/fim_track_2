from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	mobile_sensors = ['mobile_sensor_{}'.format(i) for i in range(3)]

	return LaunchDescription([
			Node(package = 'fim_track_2',
				namespace = name,
				executable = 'distributed_estimation',
				name = name,
				arguments = [name,'Odom']
				)
			for name in mobile_sensors
		])