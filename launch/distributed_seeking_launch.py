from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	mobile_sensors = ['mobile_sensor_{}'.format(i) for i in range(4)]
	execs = ['distributed_estimation','single_robot_controller','waypoint_planning']

	return LaunchDescription([
			Node(package = 'fim_track_2',
				namespace = name,
				executable = exe,
				name = name,
				arguments = [name,'Odom']
				) for exe in execs for name in mobile_sensors 
		])