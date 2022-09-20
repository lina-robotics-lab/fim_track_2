
import pickle as pkl
import numpy as np
from matplotlib import pyplot as plt

def top_n_mean(readings,n):
	"""
	top_n_mean is used to convert the reading vector of the 8 light-sensors installed
	on Turtlebots into a single scalar value, representing the overall influence of
	the light source to the Turtlebots.
	"""
	if len(readings.shape)==1:
		return readings
	rowwise_sort=np.sort(readings,axis=1)
	return np.mean(rowwise_sort[:,-n:],axis=1)

with open('calibration_data.pkl','rb') as file:
	data = pkl.load(file)



robot_loc = np.array(data['Robot'])
source_loc = np.array(data['Source'])
readings = np.array(data['Readings'])
scalar_readings = top_n_mean(readings,4)
dists = np.linalg.norm(robot_loc-source_loc,axis=-1)

with open('coefs.pkl','rb') as file:
	coefs = pkl.load(file)



d = np.linspace(np.min(dists),np.max(dists),100)


fitted_curve = coefs['k']*(d-coefs['C1'])**(coefs['b'])+coefs['C0']


plt.figure(dpi=200)

plt.scatter(dists,scalar_readings,facecolor='none',edgecolor='blue',alpha = 0.2)

plt.plot(d,fitted_curve,lw = 2)

plt.xlabel('Sensor-src Dist')
plt.ylabel('Scalar Reading')
plt.title(coefs)
plt.show()