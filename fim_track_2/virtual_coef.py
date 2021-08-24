import os
import sys
import socket

tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0,os.path.abspath(tools_root))

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_msgs.msg import Float32

import pickle as pkl

class coef_publisher(Node):
	def __init__(self,robot_namespace):
		super().__init__('coef',namespace=robot_namespace)

		C1=-0.3
		C0=0.
		b=-2.
		k=1.
		self.coefs = {'C1':C1,'C0':C0,'k':k,'b':b}
		
		_=[self.declare_parameter(name,val) for name,val in self.coefs.items()]

def main(args = sys.argv):
	rclpy.init(args=args)
	args_without_ros = rclpy.utilities.remove_ros_args(args)

	robot_namespace =''
	if len(args_without_ros)>1:
		robot_namespace = args_without_ros[1]
	else:
		print("No robot_namespace is supplied!")
	cp = coef_publisher(robot_namespace = robot_namespace)
	try:
		print("Coef Publisher Up")
		rclpy.spin(cp)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		cp.destroy_node()
		print('Coef Publisher Node Down')
		rclpy.shutdown()
if __name__ == '__main__':
	main()