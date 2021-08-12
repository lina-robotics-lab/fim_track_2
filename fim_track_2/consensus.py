import numpy as np

from functools import partial

from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

import numpy as np

class parallel_two_pass:
	"""
		Convention: neighborhood := {strict neighbors} + {myself} 
	"""
	def __init__(self,x0,N_neighborhood):
		assert(N_neighborhood!=0)

		self.data ={'x':x0,'y':1/N_neighborhood,'z':x0/N_neighborhood}
							# y is the dynamic consensus weight.

	def get_x(self):
		return self.data['x']

	def get_y(self):
		return self.data['y']

	def get_z(self):
		return self.data['z']

	def update_x(self,dx): # dx is some potential time-varying increments to be added to x. The result is a moving average through consensus.
		assert(self.get_y()!=0)
		self.data['x'] = self.get_z()/self.get_y() 
		self.data['x'] += dx.reshape(self.data['x'].shape)
		return self.get_x()

	def update_z(self,neighborhood_z):
		if len(neighborhood_z)>0:
			self.data['z'] = np.mean(neighborhood_z,axis=0)
		return self.get_z()

	def update_y(self,neighborhood_y):
		if len(neighborhood_y)>0:
			self.data['y'] = np.mean(neighborhood_y,axis=0)
		return self.get_y()

class consensus_handler:

	def __init__(self,controller_node,robot_namespace, neighborhood_namespaces,x0, topic_name='x0',qos=None):

		self.controller_node = controller_node
		self.robot_namespace = robot_namespace

		assert(robot_namespace in neighborhood_namespaces)

		if qos is None:
			qos = QoSProfile(depth=10)

		self.pass_alg = parallel_two_pass(x0,N_neighborhood = len(neighborhood_namespaces))

		self.sub_topics = ["x","y","z"]

		self.pubs ={st:controller_node.create_publisher(Float32MultiArray,'/{}/{}/{}'.format(self.robot_namespace,topic_name,st),qos) for st in self.sub_topics}

		for nb in neighborhood_namespaces:
			for t in self.sub_topics:
				controller_node.create_subscription(Float32MultiArray,'/{}/{}/{}'.format(nb,topic_name,t),partial(self.value_callback,namespace = nb,sub_topic = t),qos)

		self.neighborhood_val = {st:{nb:None for nb in neighborhood_namespaces} for st in self.sub_topics}


	def get_consensus_val(self):
		return self.pass_alg.get_x()	
		
	def value_callback(self,data,namespace,sub_topic):
		self.neighborhood_val[sub_topic][namespace] = np.array(data.data)

	
	def timer_callback(self,dx): 
		# dx is some potential time-varying increments to be added to x. The result is a moving average through consensus.
		# If the consensus is not calculating a moving average, set dx = 0.

		neighborhood_y = [y  for y in self.neighborhood_val['y'].values() if not y is None]
		neighborhood_z = [z  for z in self.neighborhood_val['z'].values() if not z is None]

		self.pass_alg.update_y(neighborhood_y)
		self.pass_alg.update_z(neighborhood_z)
		self.pass_alg.update_x(dx)

		for st in self.sub_topics:
			out = Float32MultiArray()

			out.data = list(np.array(self.pass_alg.data[st],dtype=float).ravel())

			self.pubs[st].publish(out)


