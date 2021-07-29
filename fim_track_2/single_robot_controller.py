# Single Robot Controller
# Subscribe to waypoint topic, output to cmd_vel topic

import os
import select
import sys
import termios
import tty
import threading

import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.qos import QoSProfile


tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from ros2_utils.robot_listener import robot_listener
from ros2_utils.pose import prompt_pose_type_string,turtlebot_twist, stop_twist
from actuation_control.WaypointTracking import LQR_for_motion_mimicry


def get_control_action(waypoints,curr_x):
	if len(waypoints)==0:
		return stop_twist()
		
	planning_dt = 0.1

	Q = np.array([[10,0,0],[0,10,0],[0,0,1]])
	R = np.array([[10,0],[0,1]])

	uhat,_,_ = LQR_for_motion_mimicry(waypoints,planning_dt,curr_x,Q=Q,R=R)

	[v,omega] = uhat[0]
	vel_msg=turtlebot_twist(v,omega)
	return vel_msg

def main():

	arguments = len(sys.argv) - 1
	position = 1
	# Get the robot name passed in by the user
	robot_namespace=''
	if arguments>=position:
		robot_namespace=sys.argv[position]
	
	if arguments>=position+1:
		pose_type_string = sys.argv[position+1]
	else:
		pose_type_string = prompt_pose_type_string()
		
	# Set up ROS 2 interface.
	settings = termios.tcgetattr(sys.stdin)

	rclpy.init()

	qos = QoSProfile(depth=10)
	node = rclpy.create_node('{}_controller'.format(robot_namespace))

	wp_pub = node.create_publisher(Float32MultiArray, '/{}/waypoints'.format(robot_namespace), qos)
	vel_pub = node.create_publisher(Twist, '/{}/cmd_vel'.format(robot_namespace), qos)

	rl = robot_listener(node,robot_namespace,pose_type_string)

	# Currently, ROS 2 requires such threading operation for the rate.sleep() to work properly.
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(5)
	
	waypoints = []

	# waypoints = np.array([[4,1],[5,1],[6,1]])
	
	try:
		while(rclpy.ok()):
			if len(rl.robot_loc_stack)>0:

				# Publish control actions.
				loc=rl.robot_loc_stack[-1]
				yaw=rl.robot_yaw_stack[-1]
		
				curr_x = np.array([loc[0],loc[1],yaw])			
				
				vel_pub.publish(get_control_action(waypoints,curr_x))

				# Publish the waypoints currently following.
				out=Float32MultiArray()	
				out.data=list(waypoints.ravel())
				wp_pub.publish(out)

				print("publishing")

			rate.sleep()

	except KeyboardInterrupt:
		print("Shutdown requested...exiting")

	except Exception as e:
		print(e)

	finally:
		twist = stop_twist()

		vel_pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

		rclpy.shutdown()
		thread.join()

if __name__ == '__main__':
	main()