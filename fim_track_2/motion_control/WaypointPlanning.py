import os
import sys

import numpy as np

tools_root = os.path.join(os.path.dirname(__file__))
sys.path.insert(0, os.path.abspath(tools_root))

from WaypointTracking import BURGER_MAX_LIN_VEL



def FIM_waypoints(qhat, p_0, dLdp, planning_horizon = 10, max_linear_speed = BURGER_MAX_LIN_VEL):
    
    p = p_0

    wp = [np.array(p)]

    # Gradient update
    for i in range(planning_horizon):
        dp = dLdp(qhat,p)

        p -= max_linear_speed*dp/np.linalg.norm(dp)

        wp.append(np.array(p))

    return wp