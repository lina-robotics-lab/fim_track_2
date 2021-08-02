import os
import select
import sys
import termios
import tty
import threading

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
from util_func import joint_meas_func, analytic_dhdz

class distributed_estimation_node(Node):

	def __init__(self,robot_namespace,pose_type_string,estimator,neighborhood_namespaces=None):
		super().__init__(node_name = 'estimation'.format(robot_namespace), namespace = robot_namespace)

		self.pose_type_string = pose_type_string
		self.robot_namespace = robot_namespace

		if neighborhood_namespaces is None:
			self.neighborhood_namespaces = get_sensor_names(self)
		else:
			self.neighborhood_namespaces = neighborhood_namespaces

		self.sensor_listeners = {namespace:robot_listener(self,namespace,self.pose_type_string)\
								 for namespace in neighborhood_namespaces}


		qos = QoSProfile(depth=10)

		self.z_hat_listeners = \
		{namespace:
		self.create_subscription(
			Float32MultiArray,
			'/{}/z_hat'.format(namespace),
			partial(self.z_hat_callback,namespace = namespace),
			qos)
			for namespace in neighborhood_namespaces}

		self.nb_zhats = {namespace:[] for namespace in neighborhood_namespaces}

		self.z_hat_pub = self.create_publisher(Float32MultiArray,'/{}/z_hat'.format(robot_namespace),qos)

		sleep_time = 0.5
		self.timer = self.create_timer(sleep_time,self.timer_callback)



		self.estimator = estimator

	def neighbor_h(self):

		# Hard-coded values used in development.
		C1=-0.3
		C0=0
		b=-2
		k=1

		return partial(joint_meas_func,C1,C0,k,b)
	
	def neighbor_dhdz(self):
		# Hard-coded values used in development.
		C1=-0.3
		C0=0
		b=-2
		k=1

		return partial(analytic_dhdz,C1s = C1,C0s = C0,ks = k,bs = b)

	def neighbor_zhats(self):

		return self.nb_zhats

	def z_hat_callback(self,data,namespace):
		self.nb_zhats[namespace] = np.array(data.data).flatten()

	def consensus_weights(self):
		# Temporary hard-coded equally consensus weights.
		N_neighbor = len(self.sensor_listeners)
		return np.ones(N_neighbor)/N_neighbor

	def timer_callback(self):
		p = []
		y = []
		zhat = []

		zh = self.estimator.get_z()

		for name,sl in self.sensor_listeners.items():
		
			if len(sl.robot_loc_stack)>0 and \
				 len(sl.light_reading_stack)>0:
		
					p.append(sl.get_latest_loc())
					y.append(sl.get_latest_readings())

					if len(self.nb_zhats[name])>0:
						zhat.append(self.nb_zhats[name])
					else:
						zhat.append(zh)

		self.estimator.update(self.neighbor_h(),self.neighbor_dhdz(),y,p,zhat\
								,z_neighbor_bar=None,consensus_weights=self.consensus_weights())

		print(self.robot_namespace,self.estimator.get_z(),self.consensus_weights())

		z_out = Float32MultiArray()
		z_out.data = list(zh)
		self.z_hat_pub.publish(z_out)

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
		
	


	neighborhood = set(['mobile_sensor_{}'.format(n) for n in [1,3]]+[robot_namespace])

	qhat_0 = (np.random.rand(2))*3
	estimator = ConsensusEKF(qhat_0)

	de = distributed_estimation_node(robot_namespace,pose_type_string,estimator, neighborhood_namespaces = neighborhood)
	
	try:
		print('Estimation Node Up')
		rclpy.spin(de)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		de.destroy_node()
		print('Estimation Node Down')
		rclpy.shutdown()

if __name__ == '__main__':
	main()