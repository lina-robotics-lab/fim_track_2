from launch import LaunchDescription
from launch_ros.actions import Node
import networkx as nx


def neighorhoods(G, mobile_sensors):

	G=nx.relabel_nodes(G,{i:sensor for (i,sensor) in enumerate(mobile_sensors)})
	return {sensor:list(dict(G[sensor]).keys())+[sensor] for sensor in mobile_sensors}

def generate_launch_description():
	mobile_sensors = ['MobileSensor{}'.format(i) for i in range(1,4)]

	G = nx.circulant_graph(len(mobile_sensors), [0,1])

	G.remove_edges_from(nx.selfloop_edges(G))

	nb = neighorhoods(G,mobile_sensors)


	return LaunchDescription([
			Node(package = 'fim_track_2',
				executable = 'estimate',
			arguments = [name,'optitrack',','.join(nb[name])]
				)
			for name in mobile_sensors
		])