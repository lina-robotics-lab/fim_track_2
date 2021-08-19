import time


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Bool




class NotArmedError(Exception):
    pass

TOTAL_WAITING_TIME=5

class MOVE(Node):
	def __init__(self):
		super().__init__('MOVE')
		
		qos = QoSProfile(depth=10)
		self.create_subscription(Bool,'/MISSION_CONTROL/ARM_ROBOTS',self.ARM_CALLBACK,qos)
		self.pub_ = self.create_publisher(Bool,'/MISSION_CONTROL/MOVE',10)

		self.ARM=None

		self.sleep_time = 0.1
		self.waiting_time = TOTAL_WAITING_TIME
		self.create_timer(self.sleep_time,self.timer_callback)

	def ARM_CALLBACK(self,data):
		self.ARM = data.data
		if not self.ARM:
			self.get_logger().info('Robot Disarmed.')	
			raise NotArmedError()

	def timer_callback(self):
		if self.ARM is None:
			self.waiting_time-=self.sleep_time
			if self.waiting_time<0:
				self.get_logger().info('Arming signal not received after {} seconds.'.format(TOTAL_WAITING_TIME))		
				raise NotArmedError()
		else:
			if self.ARM:
				self.move()
				print('Issuing moving command.')
			else:
				raise NotArmedError()

	def move(self):
		msg = Bool()
		msg.data = True
		self.pub_.publish(msg)

	def stop(self):
		msg = Bool()
		msg.data = False
		self.pub_.publish(msg)
		

def main():
	rclpy.init()
	move = MOVE()

	try:
		print('Waiting For Arming Signal')
		rclpy.spin(move)
	except NotArmedError:
		print('Robot Not Armed. Please arm the robots first.')
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Stopping robots...")
	finally:
		t = TOTAL_WAITING_TIME
		sleep_time = 0.1
		while t>0:
			move.stop()
			time.sleep(sleep_time)
			t-=sleep_time

		move.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()