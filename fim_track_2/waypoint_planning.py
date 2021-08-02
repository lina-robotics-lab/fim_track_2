import os
import sys

import numpy as np

from functools import partial

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node



tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from ros2_utils.robot_listener import robot_listener
from ros2_utils.pose import prompt_pose_type_string
from ros2_utils.misc import get_sensor_names

from estimation.ConsensusEKF import ConsensusEKF 
from util_func import analytic_dLdp


class waypoint_planning_node(Node):


	def __init__(self,robot_namespace,pose_type_string):
		super().__init__(node_name = 'waypoint_planning',namespace = robot_namespace)

		self.robot_namespace = robot_namespace

		qos = QoSProfile(depth=10)

		self.wp_pub = self.create_publisher(Float32MultiArray, '/{}/waypoints'.format(robot_namespace), qos)
	
		self.q_hat_sub = self.create_subscription(Float32MultiArray,'/{}/q_hat'.format(robot_namespace),self.q_hat_callback , qos)

		self.rl = robot_listener(self,robot_namespace,pose_type_string)

		sleep_time = 0.1
		self.timer = self.create_timer(sleep_time,self.timer_callback)

		# self.waypoints = []
		
		# Temporary hard-coded waypoints used in devel.	
		self.waypoints = np.array([[4.,1],[5.,1],[6.,1]])

		self.q_hat = []

		# Hard-coded values used in development.
		C1=-0.3
		C0=0
		b=-2
		k=1

		self.dLdp = partial(analytic_dLdp, C1s = C1, C0s = C0, ks=k, bs=b, FIM=None)

	def q_hat_callback(self,data):
		self.q_hat = np.array(data.data).ravel()
	
	def timer_callback(self):
		if len(self.rl.robot_loc_stack)>0 and len(self.q_hat)>0:
			# Publish control actions.
			loc=self.rl.robot_loc_stack[-1]
			yaw=self.rl.robot_yaw_stack[-1]
	
			curr_x = np.array([loc[0],loc[1],yaw])		

			# self.waypoints = FIM_waypoints(self.q_hat,loc,self.dLdp)	
			
			# Publish the waypoints currently following.
			out = Float32MultiArray()	
			print(self.waypoints.ravel())
			out.data = list(self.waypoints.ravel())
			self.wp_pub.publish(out)

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
	
	WP = waypoint_planning_node(robot_namespace,pose_type_string)

	try:
		print('Waypoint Planning Node Up')
		rclpy.spin(WP)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		WP.destroy_node()
		print('Waypoing Planning Node Down')
		rclpy.shutdown()

if __name__ == '__main__':
	main()