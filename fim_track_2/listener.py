import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class listener(Node):
	"""docstring for listener"""
	def __init__(self):
		super(listener, self).__init__('listener')
		self.sub = self.create_subscription(String,'talker/message',self.callback,10)
		self.sub

	def callback(self,msg):
		print(msg.data)
		
def main():
	rclpy.init()
	ls = listener()
	rclpy.spin(ls)

	ls.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		