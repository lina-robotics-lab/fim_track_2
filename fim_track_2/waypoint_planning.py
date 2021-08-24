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
from motion_control import WaypointPlanning

from motion_control.WaypointTracking import BURGER_MAX_LIN_VEL

from util_func import analytic_dLdp, joint_F_single
from consensus import consensus_handler


COEF_NAMES = ['C1','C0','k','b']

class waypoint_planning_node(Node):
	"""
		Convention: neighborhood := {strict neighbors} + {myself}
	"""

	def __init__(self,robot_namespace, pose_type_string, neighborhood_namespaces):
		super().__init__(node_name = 'waypoint_planning',namespace = robot_namespace)

		self.robot_namespace = robot_namespace

		assert(robot_namespace in neighborhood_namespaces)


		qos = QoSProfile(depth=10)

		self.wp_pub = self.create_publisher(Float32MultiArray, 'waypoints'.format(robot_namespace), qos)
		
		self.q_hat_sub = self.create_subscription(Float32MultiArray,'/{}/q_hat'.format(robot_namespace),self.q_hat_callback , qos)

		if neighborhood_namespaces is None:
			self.neighborhood_namespaces = get_sensor_names(self)
		else:
			self.neighborhood_namespaces = neighborhood_namespaces

		self.robot_listeners = {namespace:robot_listener(self,namespace,pose_type_string,COEF_NAMES)\
								 for namespace in neighborhood_namespaces}

		self.sleep_time = 1
		self.timer = self.create_timer(self.sleep_time,self.timer_callback)

		# self.waypoints = []
		
		# Temporary hard-coded waypoints used in devel.	
		self.waypoints = np.array([[1.,-2.5],[2.,-1.5],[3,-0.5]])
		# self.waypoints = np.array([[3,-0.5]])

		self.q_hat = []

		self.F = 0

		# FIM consensus handler
		self.FIM = np.ones((2,2))*1e-4
		self.cons = consensus_handler(self,robot_namespace,neighborhood_namespaces,self.FIM,topic_name = 'FIM',qos=qos)
	
	def list_coefs(self,coef_dicts):
		if len(coef_dicts)==0:
			# Hard-coded values used in development.	
			C1=-0.3
			C0=0
			b=-2
			k=1
		else:
			print(coef_dicts)
			C1=np.array([v['C1'] for v in coef_dicts])
			C0=np.array([v['C0'] for v in coef_dicts])
			b=np.array([v['b'] for v in coef_dicts])
			k=np.array([v['k'] for v in coef_dicts])
		return C1,C0,b,k
	
	def dLdp(self,q_hat,ps,FIM,coef_dicts=[]):

		C1,C0,b,k = self.list_coefs(coef_dicts)
		
		return analytic_dLdp(q=q_hat,ps=ps, C1s = C1, C0s = C0, ks=k, bs=b,FIM=FIM)
	
	

	def get_my_loc(self):

		return self.robot_listeners[self.robot_namespace].get_latest_loc()
	
	def get_my_coefs(self):
	
		return self.robot_listeners[self.robot_namespace].get_coefs()

	def calc_new_F(self,coef_dicts=[]):

		C1,C0,b,k = self.list_coefs(coef_dicts)

		return	joint_F_single(qhat=self.q_hat,ps=self.get_my_loc().reshape(1,-1),C1 = C1, C0 = C0, k=k, b =b)

	def q_hat_callback(self,data):
		self.q_hat = np.array(data.data).ravel()
	
	def timer_callback(self):
		
		my_loc=self.get_my_loc()
		my_coefs = self.get_my_coefs()
		# print(my_loc,my_coefs,self.q_hat)
		if (not my_loc is None) and len(my_coefs)>0 and len(self.q_hat)>0:

			# Get neighborhood locations and coefs, in matching order.
			# Make sure my_coef is on the top.

			neighbor_loc = []

			neighborhood_coefs =[]
			for name,nl in self.robot_listeners.items():
				if not name == self.robot_namespace:
					loc = nl.get_latest_loc()
					coef = nl.get_coefs()
					if (not loc is None) and (len(coef)>0):
						neighbor_loc.append(loc)
						neighborhood_coefs.append(coef)
			
			neighborhood_coefs = [self.get_my_coefs()]+neighborhood_coefs # Make sure my_coef is on the top.

			self.waypoints = WaypointPlanning.waypoints(self.q_hat,my_loc,neighbor_loc,lambda qhat,ps: self.dLdp(qhat,ps,FIM=self.FIM,coef_dicts = neighborhood_coefs), step_size = self.sleep_time * BURGER_MAX_LIN_VEL)	


			# Publish the waypoints currently following.
			out = Float32MultiArray()	
			out.data = list(self.waypoints.ravel())
			self.wp_pub.publish(out)

			# Consensus on the global FIM estimate.
			newF = self.calc_new_F()
			dF = newF-self.F
			self.cons.timer_callback(dx=dF) # Publish dF to the network.
			self.FIM = self.cons.get_consensus_val().reshape(self.FIM.shape)
			self.F = newF

			self.get_logger().info("publishing")


			# self.get_logger().info(str(self.FIM))


		# Publish the waypoints currently following.
		# out = Float32MultiArray()	
		# out.data = list(self.waypoints.ravel())
		# self.wp_pub.publish(out)


	

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

	if arguments >= position+2:
		neighborhood = set(args_without_ros[position+2].split(','))
	else:
		# neighborhood = set(['MobileSensor{}'.format(n) for n in range(1,4)])
		neighborhood = set(['MobileSensor2'])
	
	WP = waypoint_planning_node(robot_namespace,pose_type_string,neighborhood_namespaces = neighborhood)

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