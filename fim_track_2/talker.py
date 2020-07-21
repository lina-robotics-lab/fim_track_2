import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class talker(Node):
	def __init__(self):
		super().__init__('talker')
		self.pub_ = self.create_publisher(String,'talker/message',10)
		sleep_time = 0.5
		self.timer = self.create_timer(sleep_time,self.timer_callback)
	def timer_callback(self):
		msg = String()
		msg.data = 'Howdy'+str(sys.argv)

		self.pub_.publish(msg)

def main():
	rclpy.init()
	tk = talker()

	rclpy.spin(tk)

	tk.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()