'''
Author: Joshua Shaffer

Purpose: Driver function for UAV fire sim.

Created: June 5, 2018
Updated: June 13, 2018
'''

import numpy as np
import math
from Parameters import Params
from SimFunctions import simulation_loop
from FleetAndAgents import Agent, Fleet
from FireEnv import Env
from Dynamics import Dynamics
from Graph import Graph
import csv

gra = Graph(Dynamics)
gra.generate_graph_from_dynamics([1.0, 1.0, math.pi/2.0], [3, (1., 10., 0), (1., 10., 0), (0., 3.0*math.pi/2.0, 1)],
                                 [[1./3., 0.], [1.0*math.pi/6.0, math.pi/6.0], [1.0*math.pi/6.0, -math.pi/6.0],
                                  [0.0, 0.0]], 3., 0.1, 0.1)

# Populate obstacles and starting fire locations (plus intensities)
params = Params()
obstacles = np.zeros((params.width, params.height))
fires = np.zeros((params.width, params.height))

# Obstacles and fires
obs = [(2, 8), (3, 8), (3, 7), (4, 6), (6, 5), (6, 6), (6, 7), (7, 4), (7, 5), (7, 6), (8, 4), (8, 5), (8, 6), (8, 7)]
fires = [(3, 4), (3, 5), (4, 4), (7, 2), (7, 3)]

# Begin simulation
monte_carlo_number = 1  # Number of desired monte carlo simulations

for N in range(0, 3):
    params.N = N
    results = list()
    for monte in range(1, monte_carlo_number+1):
        # Environment holder
        env = Env(U=4.0, U_angle=0.0, R=18.0, starting_loc=[[50.1, 35.1]])#U=1.0, U_angle=0.0, R=3.0, starting_loc=[[50.1, 60.1]]) [35.1, 35.1][[50.1, 20.1],[50.1, 80.1]][[25.1, 45.1], [45.1, 25.1]]
        env.obs_abstraction = obs
        obstacles_list = list()
        # Populate the environment initial conditions
        for i in range(1, params.width+1):
            for j in range(1, params.height+1):
                Water_Accum = 0.0

                if (i, j) in obs:
                    Obstacle = 1
                    obstacles_list.append([((i)*10.0 - 5.0, (j)*10.0 - 5.0), ((i)*10.0 + 5.0, (j)*10.0 - 5.0),
                                           ((i) * 10.0 + 5.0, (j) * 10.0 + 5.0), ((i)*10.0 - 5.0, (j)*10.0 + 5.0)])
                    env.obs_loc_synth.append((i,j))
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

                env.add_cell((i, j), (10.0*(i-1)+5.0, 10.0*(j-1)+5.0,), 3, [10.0, 10.0])


        env.obstacles = obstacles_list
        # State setup
        # Starting states
        starts = [[2.0, 2.0, 0.0], [2.0, 2.0, math.pi/2.0], [2.0, 2.0, math.pi], [2.0, 2.0, math.pi*3.0/2.0], [2.0, 2.0, 0.0], [2.0, 3.0, 0.0]]
        fleet = Fleet(gra)

        for i in range(0, params.N):
            fleet.add_agent(Agent(starts[i], starts[i], 'UAV'+str(i+1), Dynamics, [6.0, 8.0, math.pi*3.0/2.0], 1, 100, params.stop_interval))


        results.append(simulation_loop(fleet, env, params, True))
        print('Finished monte run number: ' + str(monte) + ' at ' + str(results[monte-1]) + ' simulation seconds')









