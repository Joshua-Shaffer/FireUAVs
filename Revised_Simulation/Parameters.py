'''
Class holding all static parameters used in simulation. Shoved them in here to condense everything
'''
import numpy as np
import math

class Params(object):

    def __init__(self):
        self.N = 4  # Number of UAVs
        self.mode_version = 'Sync'
        self.width = 10
        self.height = 10
        self.partition_size = 45  # meters
        self.base_location = (2, 2)
        self.Dc_coeff = 0.1
        self.Cs_coeff = 1 - self.Dc_coeff
        self.Ef = 1
        self.Bf = 0.02
        self.Wf = 0.1
        self.switch_penalty = 10
        self.UAV_speed = 15  # m/s
        self.update_step = 3.0 #self.partition_size / self.UAV_speed  # temporary for paper purposes
        self.time_step = self.update_step / 1.0
        self.sim_time = 100000
        self.stop_interval = 4.0*60.0
        self.max_fire_intensity = 3
        self.burn_through = 10.0

        # Parameters related to visualization
        self.WIDTH = 40
        self.HEIGHT = 40
        self.MARGIN = 2
        self.MARGIN_HALF = 1
        self.TRI_BASE = 20
        self.TRI_HEIGHT = 30
        self.TRI_BACK_ANG = -math.pi + math.atan(self.TRI_BASE/self.TRI_HEIGHT)
        self.FRONT_VECTOR = [self.TRI_HEIGHT/2, 0]
        self.BACK_BOT_VECT = [-self.TRI_HEIGHT/2, -self.TRI_BASE/2]
        self.BACK_TOP_VECT = [-self.TRI_HEIGHT/2, self.TRI_BASE/2]
        self.fire_color = [(255, 238, 61), (255, 170, 61), (252, 61, 27)]
        self.obs_color = (130, 130, 130)
        self.fuel_color = [(3, 25, 6), (12, 106, 26), (16, 140, 35), (16, 190, 35), (16, 255, 35), (81, 255, 101),
                           (255, 255, 255)]  # need to choose fuel colors...

        # Parameters related to simulation
        self.sim_throttle = 1.0

    def __str__(self):
        return 'Params...'

    def __repr__(self):
        return self.__str__()
