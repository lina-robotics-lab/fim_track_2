# Single Robot Controller
# Subscribe to waypoint topic, output to cmd_vel topic
import os
import sys

import numpy as np

from functools import partial

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from ros2_utils.robot_listener import robot_listener
from ros2_utils.pose import prompt_pose_type_string,turtlebot_twist, stop_twist

from motion_control.WaypointTracking import LQR_for_motion_mimicry
# from util_func import local_dLdp

from collections import deque

def get_control_action(waypoints,curr_x):
	if len(waypoints)==0:
		return stop_twist()
		
	planning_dt = 0.1

	Q = np.array([[10,0,0],[0,10,0],[0,0,1]])
	R = np.array([[10,0],[0,1]])

	uhat,_,_ = LQR_for_motion_mimicry(waypoints,planning_dt,curr_x,Q=Q,R=R)

	return uhat
	# [v,omega] = uhat[0]
	# vel_msg=turtlebot_twist(v,omega)
	# return vel_msg


class motion_control_node(Node):

	def __init__(self,robot_namespace,pose_type_string):
		super().__init__(node_name = 'motion_control',namespace = robot_namespace)

		self.robot_namespace = robot_namespace

		qos = QoSProfile(depth=10)

		self.vel_pub = self.create_publisher(Twist, '/{}/cmd_vel'.format(robot_namespace), qos)

		self.rl = robot_listener(self,robot_namespace,pose_type_string)
		self.wp_sub = self.create_subscription(Float32MultiArray, '/{}/waypoints'.format(robot_namespace), self.waypoint_callback, qos)
	

		sleep_time = 0.1
		self.timer = self.create_timer(sleep_time,self.timer_callback)

		# self.waypoints = []
		
		# Temporary hard-coded waypoints used in devel.	
		self.control_actions = deque([])

	def waypoint_callback(self,data):
		waypoints = np.array(data.data).reshape(-1,2)

		if len(self.rl.robot_loc_stack)>0:
			loc=self.rl.robot_loc_stack[-1]
			yaw=self.rl.robot_yaw_stack[-1]

			curr_x = np.array([loc[0],loc[1],yaw])		

			self.control_actions = deque(get_control_action(waypoints,curr_x))

	def timer_callback(self):
		if len(self.control_actions)>0:
			# Pop and publish the left-most control action.
			[v,omega] = self.control_actions.popleft()

			print(v,omega)
			
			vel_msg = turtlebot_twist(v,omega)
			self.vel_pub.publish(vel_msg)
	
			print("publishing")


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
	
	if arguments >= position+1:
		pose_type_string = args_without_ros[position+1]
	else:
		pose_type_string = prompt_pose_type_string()
	
	MP = motion_control_node(robot_namespace,pose_type_string)

	try:
		print('Motion Control Node Up')
		rclpy.spin(MP)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		twist = stop_twist()

		MP.vel_pub.publish(twist)

		MP.destroy_node()
		print('Motion Control Node Down')
		rclpy.shutdown()

if __name__ == '__main__':
	main()