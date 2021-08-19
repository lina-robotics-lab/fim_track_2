# Test Spin
import os
import sys

import numpy as np


from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray,Bool
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from ros2_utils.pose import turtlebot_twist, stop_twist




class test_spin(Node):

	def __init__(self,robot_namespace):
		super().__init__(node_name = 'test_motion',namespace = robot_namespace)

		self.robot_namespace = robot_namespace

		qos = QoSProfile(depth=10)

		self.vel_pub = self.create_publisher(Twist, '/{}/cmd_vel'.format(robot_namespace), qos)

		self.move_sub = self.create_subscription(Bool,'/MISSION_CONTROL/MOVE',self.MOVE_CALLBACK,qos)
		
		sleep_time = 0.1
		self.timer = self.create_timer(sleep_time,self.timer_callback)

		self.MOVE = False

	def MOVE_CALLBACK(self,data):

		if not self.MOVE == data.data:
			if data.data:
				self.get_logger().info('Robot Moving')
			else:
				self.get_logger().info('Robot Stopping')
				self.vel_pub.publish(stop_twist())

		self.MOVE = data.data
		
	def timer_callback(self):
		if self.MOVE:
			print('Move')
			vel_msg = turtlebot_twist(0.0,1.2)
			self.vel_pub.publish(vel_msg)

			# self.MOVE=False # Crucial: reset self.MOVE to False everytime after new velocity is published, so that the robots will stop when remote loses connetion.
		else:
			self.vel_pub.publish(stop_twist())

def main(args = sys.argv):

	rclpy.init(args=args)
	args_without_ros = rclpy.utilities.remove_ros_args(args)

	print(args_without_ros)

	arguments = len(args_without_ros) - 1
	position = 1


	# Get the robot name passed in by the user
	robot_namespace=''
	if arguments >= position:
		robot_namespace = args_without_ros[position]
	
	MP = test_spin(robot_namespace)

	try:
		print('Motion Test Node Up')
		rclpy.spin(MP)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		twist = stop_twist()

		MP.vel_pub.publish(twist)

		MP.destroy_node()
		print('Motion Test Node Down')
		rclpy.shutdown()

if __name__ == '__main__':
	main()