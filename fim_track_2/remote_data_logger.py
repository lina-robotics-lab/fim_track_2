# The big distributed seeking node. This reduces the number of topic subscription needed.
import os
import sys
import traceback

import numpy as np
import pickle as pkl

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

class remote_data_logger(Node):
	def __init__(self,pose_type_string,robot_namespaces):
		super().__init__(node_name = 'remote_data_logger')
		self.pose_type_string = pose_type_string
		self.robot_namespaces = robot_namespaces

		"""
			ROS 2 topic listeners
		"""

		qos = QoSProfile(depth=10)

		self.robot_listeners = {namespace:robot_listener(self,namespace,self.pose_type_string)\
								 for namespace in self.robot_namespaces}
		self.z_hat_listeners = \
		{namespace:
		self.create_subscription(
			Float32MultiArray,
			'/{}/z_hat'.format(namespace),
			partial(self.z_hat_callback,namespace = namespace),
			qos)
			for namespace in self.robot_namespaces}

		self.FIM_listeners = \
		{namespace:
		self.create_subscription(
			Float32MultiArray,
			'/{}/FIM/x'.format(namespace),
			partial(self.FIM_callback,namespace = namespace),
			qos)
			for namespace in self.robot_namespaces}

		self.waypoint_listeners = \
		{namespace:
		self.create_subscription(
			Float32MultiArray,
			'/{}/waypoints'.format(namespace),
			partial(self.waypoint_callback,namespace = namespace),
			qos)
			for namespace in self.robot_namespaces}


		"""
		Timer initialization
		"""

		self.sleep_time = 0.1

		self.timer = self.create_timer(self.sleep_time,self.timer_callback)

		"""
			Data containers
		"""
		
		self.data_entries = ['loc','yaw','y','z_hat','FIM','waypoints']

		self.curr_val = {name:{entry:None for entry in self.data_entries} for name in self.robot_namespaces}
		self.history = {name:{entry:[] for entry in self.data_entries} for name in self.robot_namespaces}

		
		 
	def z_hat_callback(self,data,namespace):
		self.curr_val[namespace]['z_hat'] = np.array(data.data).flatten()

	def FIM_callback(self,data,namespace):
		self.curr_val[namespace]['FIM'] = np.array(data.data).reshape(-1,2)

	def waypoint_callback(self,data,namespace):
		self.curr_val[namespace]['waypoints'] = np.array(data.data).reshape(-1,2)

	def timer_callback(self):
		"""
			Data Logging
		"""
		for name in self.robot_namespaces:

			self.curr_val[name]['loc'] = self.robot_listeners[name].get_latest_loc()
			self.curr_val[name]['yaw'] = self.robot_listeners[name].get_latest_yaw()
			self.curr_val[name]['y'] = self.robot_listeners[name].get_latest_readings()

			if not np.any([v is None for v in self.curr_val[name].values()]):
				for entry in self.data_entries:
					self.history[name][entry].append(self.curr_val[name][entry])
			




def main(args=sys.argv):
	rclpy.init(args=args)
	args_without_ros = rclpy.utilities.remove_ros_args(args)

	print(args_without_ros)
	arguments = len(args_without_ros) - 1
	position = 1


	# Get the robot name passed in by the user
	robot_namespace=''
	
	if arguments >= position:
		pose_type_string = args_without_ros[position]
	else:
		pose_type_string = prompt_pose_type_string()
	
	if arguments >= position+1:
		neighborhood = set(args_without_ros[position+1].split(','))
	else:
		neighborhood = set(['MobileSensor{}'.format(n) for n in range(1,5)])
		# neighborhood = set(['MobileSensor2'])
	
	
	log = remote_data_logger(pose_type_string,neighborhood)
	
	log.get_logger().info(str(args_without_ros))
	try:
		print('Remote Logger Node Up')
		rclpy.spin(log)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		timestamp =''
		with open('distributed_seeking_data_{}.pkl'.format(timestamp),'wb') as file:
			pkl.dump(log.history,file)

		log.destroy_node()
		print('Remote Logger Node Down')
		rclpy.shutdown()

if __name__ == '__main__':
	main()