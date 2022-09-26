from launch import LaunchDescription
from launch_ros.actions import Node

import networkx as nx

def neighorhoods(G, mobile_sensors):

	G=nx.relabel_nodes(G,{i:sensor for (i,sensor) in enumerate(mobile_sensors)})
	return {sensor:list(dict(G[sensor]).keys())+[sensor] for sensor in mobile_sensors}

def generate_launch_description():
	# n_sensors = 3
	# n_sensors = 4
	n_sensors = 6
	mobile_sensors = ['MobileSensor{}'.format(i) for i in range(1,n_sensors+1)]
	
	# n_sensors = 6
	# mobile_sensors = ['MobileSensor{}'.format(i) for i in range(2,n_sensors+1)]


	G = nx.circulant_graph(len(mobile_sensors), [0,1])

	G.remove_edges_from(nx.selfloop_edges(G))

	nb = neighorhoods(G,mobile_sensors)

	print(nb)

	execs = []
	
	execs.extend([Node(package = 'fim_track_2',
		executable = 'distributed_seeking',
		arguments = [name,'optitrack',','.join(nb[name])]
		) for name in mobile_sensors ])

	return LaunchDescription(execs)