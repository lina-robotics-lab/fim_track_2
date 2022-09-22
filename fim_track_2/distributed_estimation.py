# Run the consensus EKF to estimate the source location.
import os
import sys
import traceback

import numpy as np

from functools import partial
from collections import deque

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray,Bool
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node



tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

# General dependencies
from ros2_utils.robot_listener import robot_listener
from ros2_utils.pose import prompt_pose_type_string
from ros2_utils.misc import get_sensor_names


# Estimation dependencies
from estimation.ConsensusEKF import ConsensusEKF 
from util_func import joint_meas_func, analytic_dhdz, top_n_mean


COEF_NAMES = ['C1','C0','k','b']


class distributed_estimation(Node):
	def __init__(self,robot_namespace,pose_type_string,estimator,neighborhood_namespaces=None):
		super().__init__(node_name = 'distributed_seeking', namespace = robot_namespace)
		self.pose_type_string = pose_type_string
		self.robot_namespace = robot_namespace

		assert(robot_namespace in neighborhood_namespaces)
		if neighborhood_namespaces is None:
			self.neighborhood_namespaces = get_sensor_names(self)
		else:
			self.neighborhood_namespaces = neighborhood_namespaces

		self.robot_listeners = {namespace:robot_listener(self,namespace,self.pose_type_string,COEF_NAMES)\
								 for namespace in neighborhood_namespaces}


		qos = QoSProfile(depth=10)

		"""
		Timer initialization
		"""

		# self.est_sleep_time = 0.1

		self.est_sleep_time = 2
		
		self.estimation_timer = self.create_timer(self.est_sleep_time,self.est_callback)

		""" 
		Estimation initializations 
		"""

		self.z_hat_listeners = \
		{namespace:
		self.create_subscription(
			Float32MultiArray,
			'/{}/z_hat'.format(namespace),
			partial(self.z_hat_callback,namespace = namespace),
			qos)
			for namespace in neighborhood_namespaces}

		self.nb_zhats = {namespace:[] for namespace in neighborhood_namespaces}

		self.z_hat_pub = self.create_publisher(Float32MultiArray,'z_hat',qos)
		self.q_hat_pub = self.create_publisher(Float32MultiArray,'q_hat',qos)

		self.estimator = estimator

		self.q_hat = self.estimator.get_q()

	def est_reset(self):
		self.estimator.reset()
		self.q_hat = self.estimator.get_q()

	def list_coefs(self,coef_dicts):
		if len(coef_dicts)==0:
			# Hard-coded values used in development.	
			C1=-0.3
			C0=0
			b=-2
			k=1
		else:
			C1=np.array([v['C1'] for v in coef_dicts])
			C0=np.array([v['C0'] for v in coef_dicts])
			b=np.array([v['b'] for v in coef_dicts])
			k=np.array([v['k'] for v in coef_dicts])
		return C1,C0,b,k

	def neighbor_h(self,coef_dicts=[]):
		C1,C0,b,k = self.list_coefs(coef_dicts)
			
		return partial(joint_meas_func,C1,C0,k,b)
	
	def neighbor_dhdz(self,coef_dicts=[]):
		C1,C0,b,k = self.list_coefs(coef_dicts)

		return partial(analytic_dhdz,C1s = C1,C0s = C0,ks = k,bs = b)

	def neighbor_zhats(self):

		return self.nb_zhats

	def z_hat_callback(self,data,namespace):
		self.nb_zhats[namespace] = np.array(data.data).flatten()

	def consensus_weights(self,y,p):
		# assert(len(y)==len(p))
		# # Temporary hard-coded equally consensus weights. Making sure the consensus weights sum to one.
		# N_neighbor = len(y)
		# return np.ones(N_neighbor)/N_neighbor
		return None

	def process_readings(self,readings):
		return top_n_mean(np.array(readings),4)

	

	def est_callback(self):
		""" 
				Estimation 
		"""
		
		p = []
		y = []
		zhat = []
		coefs = []

		zh = self.estimator.get_z()

		# self.get_logger().info('name:{} loc:{} reading:{} coef:{}'.format(name,loc, reading,coef))
	
		for name,sl in self.robot_listeners.items():
			# self.get_logger().info('scalar y:{}.'.format(self.process_readings(sl.get_latest_readings())))
			# print(name,sl.coef_future.done(),sl.get_coefs())	
			loc = sl.get_latest_loc()
			reading = sl.get_latest_readings()
			coef = sl.get_coefs()

			# self.get_logger().info('name:{} loc:{} reading:{} coef:{}'.format(name,loc, reading,coef))
			if (not loc is None) and \
				 (not reading is None) and\
				 	len(coef)==len(COEF_NAMES):
					p.append(loc)
					y.append(self.process_readings(reading))
					# print(self.process_readings(sl.get_latest_readings()),sl.get_latest_readings())
				
					if len(self.nb_zhats[name])>0:
						zhat.append(self.nb_zhats[name])
					else:
						zhat.append(zh)

					coefs.append(coef)

		zhat = np.array(zhat)
		# self.get_logger().info('zhat:{}. zh:{} y:{} p:{} coefs:{}'.format(zhat,zh,y,p,coefs))
		try:
			if len(p)>0 and len(y)>0 and len(zhat)>0:
				self.estimator.update(self.neighbor_h(coefs),self.neighbor_dhdz(coefs),y,p,zhat\
									,z_neighbor_bar=None,consensus_weights=self.consensus_weights(y,p))

			# Publish z_hat and q_hat
			z_out = Float32MultiArray()
			z_out.data = list(zh)
			self.z_hat_pub.publish(z_out)

			qh = self.estimator.get_q()
			q_out = Float32MultiArray()
			q_out.data = list(qh)
			self.q_hat_pub.publish(q_out)
			# self.get_logger().info('qhat:{}'.format(qh))
			self.q_hat = qh 
			self.get_logger().info('name:{} robot loc:{} qhat:{}'.format(name,loc, qh))	
		
		except ValueError as err:
			self.get_logger().info("Not updating due to ValueError")
			traceback.print_exc()


def main(args=sys.argv):
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
		neighborhood = set(['MobileSensor{}'.format(n) for n in range(1,5)])
		# neighborhood = set(['MobileSensor2'])
	
	
	# qhat_0 = (np.random.rand(2)-0.5)*0.5+np.array([2,-2])
	qhat_0 = np.array([-1,-.0])
	# estimator = ConsensusEKF(qhat_0)
	estimator = ConsensusEKF(qhat_0,0.1,\
	       # Dimensions about the lab, fixed.
            x_max = 0.0,
            x_min = -4.0,
            y_max = 0.0,
            y_min = -4.0)

	de = distributed_estimation(robot_namespace,pose_type_string,estimator, neighborhood_namespaces = neighborhood)
	
	de.get_logger().info(str(args_without_ros))
	try:
		print('Distributed Estimation Node Up')
		rclpy.spin(de)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		de.destroy_node()
		print('Distributed Estimation Node Down')
		rclpy.shutdown()

if __name__ == '__main__':
	main()