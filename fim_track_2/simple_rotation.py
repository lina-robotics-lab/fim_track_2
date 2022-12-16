# The big distributed seeking node. This reduces the number of topic subscription needed.
import os
import sys
import time

import numpy as np


from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node



tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))



# Motion control dependencies
from ros2_utils.pose import turtlebot_twist, stop_twist

def motion_callback(de):
	

		[v,omega] = [0,0.5]
		
		vel_msg = turtlebot_twist(v,omega)

		vel_pub.publish(vel_msg)


		
def main(args=sys.argv):
	rclpy.init(args=args)

	qos = QoSProfile(depth=10)
	
	de = Node('SimpleRotation')

	vel_pubs = [de.create_publisher(Twist, '/MobileSensor{}/cmd_vel'.format(n), qos) for n in [2,5]]
	motion_sleep_time = 0.1
	
	try:
		print('SimpleRotation Node Up')
		while True:
			[v,omega] = [0.5,0.5]
			
			vel_msg = turtlebot_twist(v,omega)
			for pub in vel_pubs:
				pub.publish(vel_msg)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
		for _ in range(30):# Publish consecutive stop twist for 3 seconds to ensure the robot steps.
			for pub in vel_pubs:
				pub.publish(stop_twist())
			time.sleep(0.1)
	finally:
		de.destroy_node()
		print('SimpleRotation Node Down')
		rclpy.shutdown()

if __name__ == '__main__':
	main()