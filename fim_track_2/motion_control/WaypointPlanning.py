import os
import sys

import numpy as np

tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from WaypointTracking import BURGER_MAX_LIN_VEL

# The radius of a Turtlebot Burger. Useful in collision avoidance.
BURGER_RADIUS = 0.220


def waypoints(qhat, my_loc, neighbor_loc, dLdp, planning_horizon = 10, step_size = 1.5*BURGER_MAX_LIN_VEL,free_space=None):
    
    def joint_waypoints(ps_0):
        
        ps = ps_0

        wp = [np.array(ps)]
        # wp = []

        # Gradient update
        for i in range(planning_horizon):
            dp = dLdp(qhat,ps)

            ps -= step_size*dp/np.linalg.norm(dp,axis = 1).reshape(-1,1)

            if not free_space is None: # Do projected GD if free_space region is provided.
                ps = free_space.project_point(ps)

            wp.append(np.array(ps))

        return np.array(wp)

    if len(neighbor_loc)>0:
        ps_0 = np.vstack([my_loc,neighbor_loc]) # Put the loc of myself at the first row
    else:
        ps_0 = my_loc
    wp = joint_waypoints(ps_0) # wp.shape = (planning_horizon+1,N_sensors,space_dim)

    return wp[1:,0,:] # Return only the waypoints of myself.
