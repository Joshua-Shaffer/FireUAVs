"""
OffBoard_simulator.py
Author: Joshua Shaffer
Purpose: Test computer-based mission run of the fireUAV simulator in real-time.
    This is the python script that will go on the base computer and run the main fire-fighting UAV simulation
"""

from __future__ import print_function
import time
import math
import pygame
from Fire_UAV_simulation.FireUAV_modules import Sim_Object
from dronekit import connect, LocationGlobal

# Set up option parsing to get number of vehicles connecting to the script
import argparse
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--vehicle_tot_number', help="Total number of UAVs")
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

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobal(newlat, newlon, original_location.alt)

"""'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
TCP connection section:
    This is a temporary section for setting up the necessary TCP connection on the same computer. In implementation,
    we will need to change accordingly with the WIFI or radio setup chosen for the drones
"""
TCP_IP = '127.0.0.1'
vehicles = list()
for i in range(0, int(vehicle_tot_num)):
    TCP_PORT = 5005 + i * 10
    vehicles.append(connect(connection_strings[i], wait_ready=True, baud=115200))
    print('Connection on vehicle ' + str(i + 1) + ' is at address: ' + str(addr_temp))
    if i == 0:
        origin = vehicles[i].location.global_frame
        origin = get_location_metres(origin, -SCALE_PER_UNIT*(SIM_START_LOC[0][0]-1.0),
                            -SCALE_PER_UNIT * (SIM_START_LOC[0][1] - 1.0))


for i, veh_con in enumerate(vehicles):
    vehicles[i].parameters['SWARMID'] = i+1
    vehicles[i].flush()
    vehicles[i].parameters['SCALE_GRAPH'] = SCALE_PER_UNIT
    vehicles[i].flush()
    vehicles[i].parameters['TRANS_DUR'] = REAL_TIME_UPDATE
    vehicles[i].flush()
    vehicles[i].parameters['POS_K_GAIN'] = K_p
    vehicles[i].flush()
    vehicles[i].parameters['VEL_K_GAIN'] = K_d
    vehicles[i].flush()
    vehicles[i].parameters['ORILAT'] = origin.lat
    vehicles[i].flush()
    vehicles[i].parameters['ORILONG'] = origin.long
    vehicles[i].flush()
    vehicles[i].parameters['ORIALT'] = origin.alt
    vehicles[i].flush()
    print(i)
    print('Parameter packet sent to vehicle #' + str(i + 1) + '!')

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


def update_visualizations(scale):
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
        x_loc, y_loc = get_distance_metres_xy(origin, vehicles[ll].location.global_frame)
        x_loc = x_loc/scale
        y_loc = y_loc/scale
        heading_angle = vehicles[ll].heading
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


def close_all():
    """
    Used to send the End command to all UAVs and close the TCP ports. Will need to modify such that it waits for
        all UAVs to return home successfully before closing
    :param listeners:
    :return:
    """
    for quads in vehicles:
        quads.parameters['PARAMETER_DKM'] = 10
        quads.flush()

    print('Successfully sent closed command')


def project_run():
    """
    Start up the simulation and arm vehicles to starting altitude. Start threads associated with sending information
    to each UAV and listening to each UAV
    """

    print('Priming FireUAV simulation')

    scale_per_unit = 10.0
    real_time_update = 5.0  # secs
    sim_time = 0.0
    sim_time_update = 1.0  # seconds
    sim_time_throttler = 0.1
    sim_update_number = 0

    steps_ahead = 2
    while statement is False:
        statement = True
        for quads in vehicles:
            statement = statement and quads.parameters['PARAMETER_DKM'] == 1

    #print('Sent go command and starting sim')
    real_time_start = time.time()
    while True:

        # Allow visualization function to close the simulation and vehicles down.
        if update_visualizations(scale_per_unit) is False:
            close_all()

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
                prev_key = sim_object.gra.key_to_nodes.index(prev_state)
                des_key = sim_object.gra.key_to_nodes.index(des_state)
                vehicles[i].parameters['GRAPH_DEFAULT'] = prev_key
                vehicles[i].flush()
                vehicles[i].parameters['CTRL_DEFUALT'] = des_key
                vehicles[i].flush()
                vehicles[i].parameters['TIME_DEFAULT'] = sim_time
                vehicles[i].flush()

            if sim_time == 0.0:
                for n in range(0, int(vehicle_tot_num)):
                    vehicles[i].parameters['PARAMETER_DKM'] = 2
                    vehicles[i].flush()
                print('Sent the vehicles on their way')

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


