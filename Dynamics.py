'''
Classes representing dynamic instances. Contain equation of motion and numerical update. Own separate class so
that new dynamics can be added here and instantiated through other modules for new UAV agents
'''
import numpy as np
import math
from scipy.integrate import ode


class Dynamics(object):
    def __init__(self, state=None):
        self.tau = 1
        self.state = state

    def eqn_motion(self, t, x, ctrl, dist):  # Duban's car rules
        x_dot1 = ctrl[0] * math.cos(x[2]) + dist[0]
        x_dot2 = ctrl[0] * math.sin(x[2]) + dist[1]
        x_dot3 = ctrl[1] + dist[2]
        return [x_dot1, x_dot2, x_dot3]

    def integrate_state(self, time_step, x_i, ctrl, dist, int_full=False, segment=100):
        r = ode(self.eqn_motion).set_integrator('dopri5', atol=0.00000001, rtol=0.00000001)
        r.set_initial_value(x_i).set_f_params(ctrl, dist)

        path = dict()
        path[r.t] = r.y
        while r.successful() and (r.t - time_step) < 0.0 and math.fabs(r.t - time_step) > time_step/segment/10:
            r.integrate(round(1000.0*(r.t+time_step/segment))/1000.0)
            path[r.t] = r.y

        if int_full is True:
            return r.y, path
        else:
            return r.y
