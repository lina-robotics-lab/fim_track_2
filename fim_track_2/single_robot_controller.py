# Single Robot Controller
# Subscribe to waypoint topic, output to cmd_vel topic
import os
import sys

import numpy as np

from functools import partial
from collections import deque


from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray,Bool
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from ros2_utils.robot_listener import robot_listener
from ros2_utils.pose import prompt_pose_type_string,bounded_change_update, turtlebot_twist, stop_twist

from motion_control.WaypointTracking import LQR_for_motion_mimicry

from collision_avoidance.obstacle_detector import obstacle_detector, source_contact_detector
from collision_avoidance.regions import RegionsIntersection, CircleExterior



def get_control_action(waypoints,curr_x):
	if len(waypoints)==0:
		return []
		
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
		self.pose_type_string = pose_type_string

		qos = QoSProfile(depth=10)

		self.vel_pub = self.create_publisher(Twist, '/{}/cmd_vel'.format(robot_namespace), qos)

		self.move_sub = self.create_subscription(Bool,'/MISSION_CONTROL/MOVE',self.MOVE_CALLBACK,qos)
		self.rl = robot_listener(self,robot_namespace,pose_type_string)
		self.wp_sub = self.create_subscription(Float32MultiArray, '/{}/waypoints'.format(robot_namespace), self.waypoint_callback, qos)
	

		sleep_time = 0.1
		self.timer = self.create_timer(sleep_time,self.timer_callback)

		self.MOVE = False
		self.waypoints = []

		
		# Temporary hard-coded waypoints used in devel.	
		self.control_actions = deque([])

		# Obstacles are expected to be circular ones, parametrized by (loc,radius)
		self.obstacle_detector = obstacle_detector(self)
		self.obstacles = []

		self.source_contact_detector = source_contact_detector(self)

		# current control actions
		self.v = 0.0
		self.omega = 0.0

	def MOVE_CALLBACK(self,data):

		if not self.MOVE == data.data:
			if data.data:
				self.get_logger().info('Robot Moving')
			else:
				self.get_logger().info('Robot Stopping')

		self.MOVE = data.data

	def waypoint_callback(self,data):
		# Waypoint callback typically spins at a lower frequency than the timer_callback of the single_robot_controller.
		
		self.waypoints = np.array(data.data).reshape(-1,2)

		
	def timer_callback(self):
		if self.MOVE:
		# if True:
			if self.source_contact_detector.contact():
				self.vel_pub.publish(stop_twist())
				self.get_logger().info('Source Contact')
			else:

				# Project waypoints onto obstacle-free spaces.
				self.obstacles = self.obstacle_detector.get_obstacles()
				
				free_space = RegionsIntersection([CircleExterior(origin,radius) for (origin,radius) in self.obstacles])

				loc = self.rl.get_latest_loc()
				yaw = self.rl.get_latest_yaw()
				if (not loc is None) and (not yaw is None) and len(self.waypoints)>0:
					curr_x = np.array([loc[0],loc[1],yaw])		
					self.control_actions = deque(get_control_action(free_space.project_point(self.waypoints),curr_x))
				
				if len(self.control_actions)>0:

					# Pop and publish the left-most control action.

					[v,omega] = self.control_actions.popleft()
					
					[v,omega] = bounded_change_update(v,omega,self.v,self.omega) # Get a vel_msg that satisfies max change and max value constraints.
					
					vel_msg = turtlebot_twist(v,omega)

					# Update current v and omega
					self.v = v
					self.omega = omega

					self.vel_pub.publish(vel_msg)
				else:
					self.vel_pub.publish(stop_twist())
					print("Running out of control actions.")

					# Update current v and omega
					self.v = 0.0
					self.omega = 0.0

		else:
			self.vel_pub.publish(stop_twist())
			# Update current v and omega
			self.v = 0.0
			self.omega = 0.0
		# self.MOVE=False # Crucial: reset self.MOVE to False everytime after new velocity is published, so that the robots will stop when remote loses connetion.
	

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