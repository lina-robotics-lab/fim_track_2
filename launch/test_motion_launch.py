from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	mobile_sensors = ['MobileSensor{}'.format(i) for i in range(1,4)]

	return LaunchDescription([
			Node(package = 'fim_track_2',
				executable = 'test_motion',
				arguments = [name]
				)
			for name in mobile_sensors
		])