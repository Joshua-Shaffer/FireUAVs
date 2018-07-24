'''
Author: Joshua Shaffer

Purpose: Driver function for UAV fire sim. Will be retooled to work with the DroneKit on real-time basis

Created: June 5, 2018
Updated: June 23, 2018
'''

import math
from Parameters_dronekit_version import Params
from FleetAndAgents import Agent, Fleet
from FireEnv import Env, Cell
from Dynamics import Dynamics
from Graph import Graph

class Sim_Object(object):

    def __init__(self, start_loc=None, N=1, mode_version='NonSync', update_step=1.0, segment_per_update=10.0,
                 segment_interval=100):

        self.gra = Graph(Dynamics)
        self.segment_interval = segment_interval
        self.gra.generate_graph_from_dynamics([1.0, 1.0, math.pi/2.0], [3, (1., 10., 0), (1., 10., 0), (0., 3.0*math.pi/2.0, 1)],
                                     [[1., 0.], [1.0*math.pi/2.0, math.pi/2.0], [1.0*math.pi/2.0, -math.pi/2.0],
                                      [0.0, 0.0]], 1., 0.1, 0.1, self.segment_interval)


        # Populate obstacles and starting fire locations (plus intensities)
        self.params = Params(N, mode_version, update_step, segment_per_update)

        # Obstacles and fires
        obs = [(2, 8), (3, 8), (3, 7), (4, 6), (6, 5), (6, 6), (6, 7), (7, 4), (7, 5), (7, 6), (8, 4), (8, 5), (8, 6), (8, 7)]
        fires = [(3, 4), (3, 5), (4, 4), (7, 2), (7, 3)]

        # Environment holder
        self.env = Env()

        # Populate the environment initial conditions
        for i in range(1, self.params.width+1):
            for j in range(1, self.params.height+1):
                Water_Accum = 0.0

                if (i, j) in obs:
                    Obstacle = 1
                    Fuel = 0
                else:
                    Obstacle = 0
                    Fuel = 6

                if (i, j) in fires:
                    Fires = 2
                    FireUpdate = 0
                else:
                    Fires = 0
                    FireUpdate = 0

                self.env.add_cell((i, j), self.params, Obstacle, Fuel, Fires, FireUpdate, Water_Accum)

        # State setup
        # Starting states
        if start_loc is None:
            starts = [[2.0, 2.0, 0.0], [2.0, 2.0, math.pi/2.0], [2.0, 1.0, math.pi], [1.0, 2.0, math.pi*3.0/2.0], [2.0, 3.0, 0.0], [2.0, 3.0, 0.0]]
        else:
            starts = start_loc

        self.fleet = Fleet(self.gra)

        for i in range(0, self.params.N):
            self.fleet.add_agent(Agent(starts[i], starts[i], 'UAV'+str(i+1), Dynamics, [6.0, 8.0, math.pi*3.0/2.0], 1, 100, self.params.stop_interval))

    def synth_update(self, time, update_step):
        self.fleet.allocate(self.env, self.params)
        self.fleet.update_ctrls(self.env, time, self.params)
        self.env.update_cells(self.params, update_step)
        self.env.update_cells_agent_action(self.params)

        agent_truth_state = list()
        for i in self.fleet.agents:
            agent_truth_state.append(self.fleet.agents[i].state_truth)
        return agent_truth_state

    def update(self, update_step_throttle):
        self.fleet.update(self.env, self.params, update_step_throttle)

