import os
import sys

tools_root = os.path.join(".."+os.path.dirname(__file__))
print(tools_root)
sys.path.insert(0, os.path.abspath(tools_root))


import numpy as np

from ros2_utils.robot_listener import robot_listener
from ros2_utils.misc import get_sensor_names,get_source_names
from collision_avoidance import regions

# The radius of a Turtlebot Burger. Useful in collision avoidance.
BURGER_RADIUS = 0.150

class obstacle_detector:
	def __init__(self,mc_node):
		self.obs_names = get_sensor_names(mc_node)
		self.ol = [robot_listener(mc_node,name,mc_node.pose_type_string) for name in self.obs_names if not name == mc_node.robot_namespace]

	def get_obstacles(self):
		return [(l.get_latest_loc(),2.5*BURGER_RADIUS) for l in self.ol if not l.get_latest_loc() is None]
		 


class source_contact_detector:
	def __init__(self,mc_node):
		# self.src_names = get_source_names(mc_node)
		self.src_names = ['Source0']
		self.sl = [robot_listener(mc_node,name,mc_node.pose_type_string) for name in self.src_names]
		self.mc_node = mc_node

	def contact(self):

		sensor_loc = self.mc_node.get_my_loc()
		
		if sensor_loc is None:
			return False
		else:
			src_loc = [l.get_latest_loc() for l in self.sl if not l.get_latest_loc() is None]
			if len(src_loc)==0:
				return False

			src_loc = np.array(src_loc).reshape(-1,len(sensor_loc))
			
			# self.mc_node.get_logger().info(str(src_loc)+','+str(sensor_loc)+str(self.src_names))
			return np.any(np.linalg.norm(src_loc-sensor_loc)<=2.5*BURGER_RADIUS)
