import os
import sys

import numpy as np
import pickle as pkl

import rclpy
from rclpy.node import Node

tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))


from util_func import calibrate_meas_coef 
from ros2_utils.robot_listener import robot_listener
from ros2_utils.pose import prompt_pose_type_string



class calibration(Node):
	def __init__(self,robot_namespace,source_namespace,pose_type_string,save_data=False,fit_type='light_readings'):

		print(robot_namespace,source_namespace)
		super().__init__(node_name = 'calibration', namespace = robot_namespace)

		self.robot =  robot_listener(self,robot_namespace,pose_type_string)
		self.source = robot_listener(self,source_namespace,pose_type_string)

		self.robot_locs = []
		self.source_locs = []
		self.readings = []

		awake_freq=10
		self.create_timer(1/awake_freq,self.timer_callback)

		self.fit_type = fit_type
		
	
	def timer_callback(self):

		
		data = [self.robot.get_latest_loc(),self.robot.get_latest_readings(),self.source.get_latest_loc()]
		# print(data)
		if not any([d is None for d in data]):
			self.robot_locs.append(data[0])
			self.readings.append(data[1])
			self.source_locs.append(data[2])
			
	
	def calc_coefs(self):
		return calibrate_meas_coef(np.array(self.robot_locs),
							np.array(self.source_locs),
							np.array(self.readings),fit_type=self.fit_type)
		
		


def main(args=sys.argv):
	rclpy.init(args=args)
	args_without_ros = rclpy.utilities.remove_ros_args(args)

	print(args_without_ros)
	arguments = len(args_without_ros) - 1
	position = 1


	# Get the robot name passed in by the user
	robot_namespace=""
	source_namespace = ""

	if arguments >= position:
		robot_namespace = args_without_ros[position]
	else:
		print('robot_namespace cannot be empty!')
		return 1
	
	if arguments >= position+1:
		source_namespace = args_without_ros[position+1]
	else:
		print('source_namespace cannot be empty!')
		return 1
	
	if arguments >= position+2:
		pose_type_string = args_without_ros[position+2]
	else:
		pose_type_string = prompt_pose_type_string()
	
	
	cl = calibration(robot_namespace,source_namespace,pose_type_string)
	
	try:
		print('Coefficient Calibration Node Up')
		rclpy.spin(cl)
	except KeyboardInterrupt:
		print("Keyboard Interrupt. Shutting Down...")
	finally:
		# Save the raw data to files.
		data = {'Robot':cl.robot_locs,'Source':cl.source_locs,'Readings':cl.readings}
		with open('calibration_data.pkl','wb+') as file:
			pkl.dump(data,file)

		# Save the calibrated coefficients to files.
		cof = cl.calc_coefs()
		print('Coefficients:',cof)
		data = {'C1':cof[0],'C0':cof[1],'k':cof[2],'b':cof[3]}
		with open('coefs.pkl','wb+') as file:
			pkl.dump(data,file)


		cl.destroy_node()
		print('Coefficient Calibration Node Down')
		rclpy.shutdown()
	return 0

if __name__ == '__main__':
	main()