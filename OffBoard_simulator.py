"""
OffBoard_simulator.py
Author: Joshua Shaffer
Purpose: Test computer-based mission run of the fireUAV simulator in real-time.
    This is the python script that will go on the base computer and run the main fire-fighting UAV simulation
"""

from __future__ import print_function
import time
import math
import socket
import pygame
from threading import Thread
from FireUAV_modules import Sim_Object

# Set up option parsing to get number of vehicles connecting to the script
import argparse
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--vehicle_tot_number', help="ID of vehicle that this script is tied to...")
args = parser.parse_args()

vehicle_tot_num = args.vehicle_tot_number

# Visualization parameters tied to size of display
WIDTH = 40
HEIGHT = 40
MARGIN_HALF = 2

# Global parameters associated with operating the UAVs and managing differences between real-time control
# and simulated control
SCALE_PER_UNIT = 10.0  # real_meters:simulated_meters
REAL_TIME_UPDATE = 5.0  # real_secs:simulated_secs
SIM_TIME_UPDATE = 1.0  # update time associated with synthesized controllers
SIM_START_LOC = [[2.0, 2.0, 0.0], [2.0, 2.0, math.pi/2.0], [2.0, 1.0, math.pi], [1.0, 2.0, math.pi*3.0/2.0],
                 [2.0, 3.0, 0.0], [2.0, 3.0, 0.0]]
VEHICLE_COLORS = [[255, 40, 3], [36, 195, 55]]
K_p = 1.0  # Proportional and derivative gains on position
K_d = 0.1

# Create the simulation object used for managing fire sim
sim_object = Sim_Object(start_loc=SIM_START_LOC, N=int(vehicle_tot_num), mode_version='NonSync', update_step=SIM_TIME_UPDATE)


"""'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
TCP connection section:
    This is a temporary section for setting up the necessary TCP connection on the same computer. In implementation,
    we will need to change accordingly with the WIFI or radio setup chosen for the drones
"""
TCP_IP = '127.0.0.1'
s = list()
conn = list()
addr = list()
for i in range(0, int(vehicle_tot_num)):
    TCP_PORT = 5005 + i * 10
    print(TCP_PORT)
    BUFFER_SIZE = 50
    s.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
    s[i].bind((TCP_IP, TCP_PORT))
    s[i].listen(1)
    conn_temp, addr_temp = s[i].accept()
    conn.append(conn_temp)
    addr.append(addr_temp)
    print('Connection on vehicle ' + str(i + 1) + ' is at address: ' + str(addr_temp))

for i in range(0, int(vehicle_tot_num)):
    print(i)
    conn[i].send(str(SCALE_PER_UNIT) + " " + str(REAL_TIME_UPDATE) + " " + str(K_p) + " " + str(K_d) + " " +
                 str(1.0 - SIM_START_LOC[i][1]) + " " + str(1.0 - SIM_START_LOC[i][0]) + " ")
                               # This packet corresponds to the scale (real_meters:simulated_meters), and the origin
                               # location (x, y) relative to the starting location of the vehicle in the simulation
                               # units
    print('Parameter packet sent to vehicle #' + str(i + 1) + '!')

origin = conn[0].recv(BUFFER_SIZE)
print(origin)
origin = origin.split()
for i, dat in enumerate(origin):
    print(dat)
    if dat == 'None':
        origin[i] = None
    else:
        origin[i] = float(dat)

"""'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''"""


def get_distance_metres_xy(a_location1, a_location2):
    """
    Returns the ground distance in metres between two arrays with items of LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = a_location2[0] - a_location1[0]
    dlong = a_location2[1] - a_location1[1]
    return dlong * 1.113195e5, dlat * 1.113195e5


def update_visualizations(scale, listeners):
    """
    Visualization function used in main function loop. This is the ground software's understanding of the vehicles'
        locations and the desired location of the simulation
    :param scale: (real_meters:simulated_meters)
    :param listeners: List of threads tied to listening process for each simulated vehicle
    :return: True if user doesn't indicate need to exit simulation, False if user is attempting to quit...
    """
    params = sim_object.params
    env = sim_object.env
    fleet = sim_object.fleet

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False

    for row in range(params.height):
        for column in range(params.width):

            if env.cells[(column + 1, params.width - (row))].fire > 0:
                color = params.fire_color[env.cells[(column + 1, params.width - (row))].fire - 1]
            elif env.cells[(column + 1, params.width - (row))].obstacle == 1:
                color = params.obs_color
            else:
                color = params.fuel_color[env.cells[((column + 1), params.width - (row))].fuel]

            pygame.draw.rect(screen, color,
                             [(params.WIDTH) * column,
                              (params.HEIGHT) * (row), params.WIDTH,
                              params.HEIGHT])
            pygame.draw.rect(screen, (0, 0, 0),
                             [(params.WIDTH) * column,
                              (params.HEIGHT) * (row), params.WIDTH,
                              params.HEIGHT], params.MARGIN_HALF)

    for ll in fleet.agents:
        pygame.draw.polygon(screen, (94, 154, 249), fleet.agents[ll].display_loc(params))

    for ll in range(0, int(vehicle_tot_num)):
        x_loc, y_loc = get_distance_metres_xy(origin, listeners[ll].vehicle_loc)
        x_loc = x_loc/scale
        y_loc = y_loc/scale
        heading_angle = listeners[ll].vehicle_loc[3]
        for r in range(0,4):
            current_leg = heading_angle + r*90.0 + 45.0
            x_dist = math.sin(current_leg*math.pi/180.0) * WIDTH/3
            y_dist = math.cos(current_leg*math.pi/180.0) * WIDTH/3
            pygame.draw.circle(screen,
                               (VEHICLE_COLORS[ll][0], VEHICLE_COLORS[ll][1], VEHICLE_COLORS[ll][2]),
                        [int(round(WIDTH * x_loc + WIDTH / 2)) + int(round(x_dist)),
                            int(round(HEIGHT * (params.height - y_loc) - HEIGHT / 2)) -
                         int(round(y_dist))], 5)

    # Insert visualization update here
    # Limit to 60 frames per second
    clock.tick(20)

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()
    return True


class ListenToUpdates(Thread):
    """
    Thread associated with listening to a single UAV
    """
    def __init__(self, r):
        Thread.__init__(self)
        self.i = r
        self.end_sim = False  # Indicator tied to UAVs ability to end simulation from on_board process
        self.confirmation = False  # Confirmation echo of UAVs ability to start
        self.vehicle_loc = [0.0, 0.0, 0.0, 0.0]  # Dummy state of UAV, should update quickly

    def run(self):
        while True:

            data = conn[self.i].recv(BUFFER_SIZE)
            if not data:
                self.end_sim = True
                break
            elif data == 'Echoed Start!':
                self.confirmation = True
            else:
                data = data.split()
                loc = self.vehicle_loc
                for n, data_unit in enumerate(data):
                    #print(data)
                    if data_unit == 'VehLoc' and len(data) > n + 5:
                        if data[n+1] == 'None' or data[n+1] is None:
                            data[n+1] = None
                        else:
                            data[n+1] = float(data[n+1])
                        if data[n+2] == 'None' or data[n+2] is None:
                            data[n+2] = None
                        else:
                            data[n+2] = float(data[n+2])
                        if data[n+3] == 'None' or data[n+3] is None:
                            data[n+3] = None
                        else:
                            data[n+3] = float(data[n+3])
                        if data[n+4] == 'None' or data[n+4] is None:
                            data[n+4] = None
                        else:
                            data[n+4] = float(data[n+4])
                        loc = [data[n+1], data[n+2], data[n+3], data[n+4]]
                    elif data_unit == 'Echoed Start!':
                        self.confirmation = True
                        break
                    elif data_unit == 'End It!':
                        self.end_sim = True
                        break
                    self.vehicle_loc = loc


def close_all(listeners):
    """
    Used to send the End command to all UAVs and close the TCP ports. Will need to modify such that it waits for
        all UAVs to return home successfully before closing
    :param listeners:
    :return:
    """
    for ll in range(0, int(vehicle_tot_num)):
        # TODO: figure out how to end the "listener" thread and wait until confirmation is received from UAV
        conn[ll].send("End!")
        data = conn[ll].recv(BUFFER_SIZE)
        data = data.split()
        if data == 'Confirmed':
            conn[ll].close()
            conn[ll] = 'Success'

    for r in conn:
        if r != 'Success':
            print('Unsuccessful closing of all TCP ports')
            return

    print('Successful closing of all TCP ports')


def project_run():
    """
    Start up the simulation and arm vehicles to starting altitude. Start threads associated with sending information
    to each UAV and listening to each UAV
    """

    # Begin threads for listening to each UAV, place in list
    print('Starting threads for listening to updates...')
    listeners = list()
    for ll in range(0, int(vehicle_tot_num)):
        listeners.append(ListenToUpdates(ll))
        listeners[ll].start()
    print('Priming FireUAV simulation')

    scale_per_unit = 10.0
    real_time_update = 5.0  # secs
    sim_time = 0.0
    sim_time_update = 1.0  # seconds
    sim_time_throttler = 0.1
    sim_update_number = 0

    steps_ahead = 2
    for ll in range(0, int(vehicle_tot_num)):
        conn[ll].send('Go!')

    waiter = False
    while waiter is False:
        waiter = True
        for ll in range(0, int(vehicle_tot_num)):
            waiter = waiter and listeners[ll].confirmation

    print('Sent go command and starting sim')
    real_time_start = time.time()
    while True:

        # Allow visualization function to close the simulation and vehicles down.
        if update_visualizations(scale_per_unit, listeners) is False:
            close_all(listeners)

        # Allow each UAV to send end sim to base and shut everything down
        for listener in listeners:
            if listener.end_sim is True:
                close_all(listeners)
                return

        # Use of "continue" in this statement is here to make sure UAV simulation doesn't go to far ahead
        real_time = time.time() - real_time_start
        if sim_time/sim_time_update - real_time/real_time_update > steps_ahead:
            continue

        tar = divmod(sim_time, sim_time_update)
        # Check statement is used to verify the the simulated time is at the update step time in the synthesized
        # controllers (a bit convoluted due to nature of floating points in python, divmod remainder can either be
        # exactly 0.0, just a little greater than 0.0, or just under the full dividing number (e.g. 0.49999 when
        # dividing by 0.5)
        if math.fabs(tar[1]) < 0.1 * sim_time_throttler or tar[1] == 0.0 or math.fabs(
                tar[1] - sim_time_update) < 0.1 * sim_time_throttler:
            # Update sim objects synthesized controllers and environmental model
            sim_object.synth_update(sim_time, sim_time_update)
            sim_update_number = sim_update_number + 1

            # Run through all vehicles and update the queue for each one
            for n in range(0, int(vehicle_tot_num)):

                # Grab all update objects in simulation for sending to the UAVs
                prev_state = sim_object.fleet.agents['UAV' + str(n + 1)].prev_state
                des_state = sim_object.fleet.agents['UAV' + str(n + 1)].desired_state
                controls = sim_object.fleet.agents['UAV' + str(n + 1)].control_inputs
                # Update queue to each UAV
                conn[n].send(str(sim_time) + ' ' +
                             str(prev_state[0]) + ' ' + str(prev_state[1]) + ' ' + str(prev_state[2]) + ' ' +
                             str(controls[0]) + ' ' + str(controls[1]) + ' ' +
                             str(des_state[0]) + ' ' + str(des_state[1]) + ' ' + str(des_state[2]))

                # TODO: could add a feedback on the UAV informing the simulation
                #       that no next queues were available for X update steps
                # TODO: THIS COULD BE AN OPTION IS SOFTWARE CONCURRENCE (VERIFY IT)

            print('Updated step in simulation and sent all information to the UAVs')

        # Updated physical model of the sim object
        sim_object.update(sim_time_throttler)
        # Update sim time associated with sim object
        sim_time = sim_time + sim_time_throttler


"""
script start
"""
# Initialize pygame for displaying simulation
pygame.init()
# Connect to the Vehicle
window_size = [(WIDTH * sim_object.params.width),
               (HEIGHT * sim_object.params.height)]
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption("UAVs on Fire")
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
# Run the project
project_run()


