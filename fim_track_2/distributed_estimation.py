import os
import select
import sys
import termios
import tty
import threading

import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.qos import QoSProfile




tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from ros2_utils.robot_listener import robot_listener
from ros2_utils.pose import prompt_pose_type_string
from ros2_utils.misc import get_sensor_names

def main():

	arguments = len(sys.argv) - 1
	position = 1
	# Get the robot name passed in by the user
	robot_namespace=''
	if arguments>=position:
		robot_namespace=sys.argv[position]
	
	if arguments>=position+1:
		pose_type_string = sys.argv[position+1]
	else:
		pose_type_string = prompt_pose_type_string()
		
	# Set up ROS 2 interface.
	settings = termios.tcgetattr(sys.stdin)

	rclpy.init()

	qos = QoSProfile(depth=10)
	node = rclpy.create_node('{}_estimation'.format(robot_namespace))

	# neighbor_names = get_sensor_names(node)
	neighborhood = set(['mobile_sensor_{}'.format(n) for n in [1,3]]+[robot_namespace])


	rls = []
	for namespace in neighborhood:
		rls.append(robot_listener(node,namespace,pose_type_string))

	# Currently, ROS 2 requires such threading operation for the rate.sleep() to work properly.
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(5)
	
	try:
		while(rclpy.ok()):
			for rl in rls:
				if len(rl.robot_loc_stack)>0:
					#
					loc = rl.get_latest_loc()
					readings = rl.get_latest_readings()
					print(rl.robot_name,loc,readings)
				
			rate.sleep()

	except KeyboardInterrupt:
		print("Shutdown requested...exiting")

	except Exception as e:
		print(e)

	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

		rclpy.shutdown()
		thread.join()

if __name__ == '__main__':
	main()