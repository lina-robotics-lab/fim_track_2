import os
import sys

import numpy as np

tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from WaypointTracking import BURGER_MAX_LIN_VEL

def waypoints(qhat, my_loc, neighbor_loc, dLdp, planning_horizon = 10, step_size = 1.5*BURGER_MAX_LIN_VEL):
    
    def joint_waypoints(ps_0):
        
        ps = ps_0

        wp = [np.array(ps)]
        # wp = []

        # Gradient update
        for i in range(planning_horizon):
            dp = dLdp(qhat,ps)

            ps -= step_size*dp/np.linalg.norm(dp,axis = 1).reshape(-1,1)

            wp.append(np.array(ps))

        return np.array(wp)


    ps_0 = np.vstack([my_loc,neighbor_loc]) # Put the loc of myself at the first row

    wp = joint_waypoints(ps_0) # wp.shape = (planning_horizon+1,N_sensors,space_dim)

    return wp[:,0,:] # Return only the waypoints of myself.
