import rclpy
from rclpy.qos import QoSProfile

from std_msgs.msg import Float32MultiArray

from ros2_utils.pose import get_pose_type_and_topic,toxy,toyaw

from collections import deque

class robot_listener:
	''' Robot location and light_reading listener+data container.'''
	def __init__(self,controller_node,robot_namespace,pose_type_string="",max_record_len=10):
		"""
			pose_type_string is one in ["turtlesimPose", "Pose", "Odom", "optitrack"]
		"""
		self.robot_name=robot_namespace

		print('initializing {} listener'.format(robot_namespace))
		
		
		self.pose_type,self.rpose_topic=get_pose_type_and_topic(pose_type_string,robot_namespace)
		
		self.light_topic="/{}/sensor_readings".format(robot_namespace)
		self.coefs_topic="/{}/sensor_coefs".format(robot_namespace)
		self.robot_pose=None
		self.light_readings=None

		self.robot_loc_stack=deque(maxlen=max_record_len)
		self.robot_yaw_stack=deque(maxlen=max_record_len)
		self.light_reading_stack=deque(maxlen=max_record_len)
		self.rhats=deque(maxlen=max_record_len)

		self.C1=None
		self.C0=None
		self.k=None
		self.b=None

		qos = QoSProfile(depth=10)

		controller_node.create_subscription(self.pose_type, self.rpose_topic,self.robot_pose_callback_,qos)
		controller_node.create_subscription(Float32MultiArray,self.light_topic, self.light_callback_,qos)
		controller_node.create_subscription(Float32MultiArray,self.coefs_topic, self.sensor_coef_callback_,qos)


	def sensor_coef_callback_(self,data):
		coefs=data.data
		self.C1,self.C0,self.k,self.b=coefs

	def robot_pose_callback_(self,data):
		self.robot_pose=data
		self.robot_loc_stack.append(toxy(self.robot_pose))
		self.robot_yaw_stack.append(toyaw(self.robot_pose))

	def light_callback_(self,data):
		self.light_readings=data.data

