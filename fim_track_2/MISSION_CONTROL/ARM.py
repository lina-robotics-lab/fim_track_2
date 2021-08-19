import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile



class ARM(Node):
	def __init__(self):
		super().__init__('ARM')
		qos = QoSProfile(depth=10)
		self.pub_ = self.create_publisher(Bool,'/MISSION_CONTROL/ARM_ROBOTS',qos)
		sleep_time = 0.5
		self.timer = self.create_timer(sleep_time,self.timer_callback)
	def timer_callback(self):
		print("armed")
		self.arm()

	def arm(self):
		msg = Bool()
		msg.data = True
		self.pub_.publish(msg)

	def disarm(self):
		msg = Bool()
		msg.data = False
		self.pub_.publish(msg)
		

def main():
	rclpy.init()
	arm = ARM()
	try:
		print('Arming Robots')
		rclpy.spin(arm)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Disarming robots...")
	finally:
		arm.disarm()
		arm.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()