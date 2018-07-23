"""
OnBoard_controller.py
Author: Joshua Shaffer
Date: July 23, 2018
Purpose: Drone-kit python script that communicates with control commands received from the ground.
        Implements the dynamic graph constructed with a feedback/feedforward control loop
"""

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
import socket
from pymavlink import mavutil
from threading import Thread
from FireUAV_modules import Sim_Object

#  Set up option parsing to get connection string and the associated vehicle ID used
#  Used for testing in a terminal, needs to be initialized from bash script with appropriate arguments
import argparse
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--vehicle_id',help="ID of vehicle that this script is tied to...")
args = parser.parse_args()

connection_string = args.connect
vehicle_id = args.vehicle_id
sitl = None


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
TCP_PORT = 5005 + (int(vehicle_id) - 1)*10
print(TCP_PORT)
time.sleep(10)
BUFFER_SIZE = 1024
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Waiting to connect to the base server...')
s.connect((TCP_IP, TCP_PORT))
# TODO: insert time out error if connection cannot be made
print('Connected to base!')

# TODO: Parameters that must be grabbed from the base, need to check that they make sense...
data = s.recv(BUFFER_SIZE)
data = data.split()
SCALE = float(data[0])
TRANSITION_DURATION = float(data[1])
SEGMENT_CONTROL_INTERVAL = 100
K_p = float(data[2])
K_d = float(data[3])

# Temporary, should modify to work on native build of SITL if we wish to test multiple vehicles
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = 'tcp:127.0.0.1:57' + str(60 + (0) * 10)  # '127.0.0.1:145' + str(50 + (i)*10)
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True, baud=115200)
    loc_true_origin = get_location_metres(vehicle.location.global_frame, SCALE * float(data[4]), SCALE * float(data[5]))
    origin = loc_true_origin
    print('Calculated origin location')
    print(origin)

print('Sending origin to base!')
s.send(str(origin.lat) + ' ' + str(origin.lon) + ' ' + str(origin.alt))
"""'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''"""


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_distance_metres_xy(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return dlong * 1.113195e5, dlat * 1.113195e5


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint == 0:
        return None
    missionitem = vehicle.commands[nextwaypoint - 1]  # commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def ctrl_loop(ctrl_queue, timer, real_time):
    """
    Update loop for the real-time controller, tracks a reference (need to store instead
    of integrating each time...)
    :param ctrl_queue: List of starting simulation locations, simulation controls, and simulation end locations
        at a given simulation time. The dynamic graphs of the UAV and simulation must match for this to work...
    :param timer: Update time associated at this control duration
    :param real_time: Real time from outer loop
    :return:
    """
    passed_time = real_time - timer
    t_div = divmod(passed_time/TRANSITION_DURATION, 1.0/SEGMENT_CONTROL_INTERVAL)
    time_input = t_div[0]
    time_input = round(time_input)

    # grab start_loc from the queue
    start_loc = ctrl_queue[timer/TRANSITION_DURATION][1:4]
    control_inputs = ctrl_queue[timer/TRANSITION_DURATION][4:6]
    next_loc = ctrl_queue[timer/TRANSITION_DURATION][6:9]
    next_loc[0] = round(next_loc[0])
    next_loc[1] = round(next_loc[1])

    if abs(round(next_loc[2] - math.pi/2.0)) < 0.001:
        next_loc[2] = math.pi/2.0
    elif abs(round(next_loc[2] - math.pi)) < 0.001:
        next_loc[2] = math.pi
    elif abs(round(next_loc[2] - 3.0 * math.pi / 2.0)) < 0.001:
        next_loc[2] = 3.0 * math.pi / 2.0
    elif abs(round(next_loc[2] - 2.0 * math.pi)) < 0.001 or abs(round(next_loc[2] - 0.0)) < 0.001:
        next_loc[2] = 0.0

    start_loc[0] = round(start_loc[0])
    start_loc[1] = round(start_loc[1])

    if abs(round(start_loc[2] - math.pi/2.0)) < 0.001:
        start_loc[2] = math.pi/2.0
    elif abs(round(start_loc[2] - math.pi)) < 0.001:
        start_loc[2] = math.pi
    elif abs(round(start_loc[2] - 3.0 * math.pi / 2.0)) < 0.001:
        start_loc[2] = 3.0 * math.pi / 2.0
    elif abs(round(start_loc[2] - 2.0 * math.pi)) < 0.001 or abs(round(start_loc[2] - 0.0)) < 0.001:
        start_loc[2] = 0.0

    desired_state = sim_object.gra.graph[(start_loc[0], start_loc[1], start_loc[2])].children \
        [(next_loc[0], next_loc[1], next_loc[2])][2][round(SEGMENT_CONTROL_INTERVAL*(round(time_input)/SEGMENT_CONTROL_INTERVAL))/SEGMENT_CONTROL_INTERVAL]
    heading_angle = 90 - desired_state[2] * 180.0 / math.pi
    current_state = get_distance_metres_xy(origin, vehicle.location.global_frame)
    pos_ctrl_dif = ((desired_state[0] - 1.0) * SCALE - current_state[0], (desired_state[1] - 1.0) * SCALE - current_state[1])
    current_vel = (math.sin(vehicle.heading / 180.0 * math.pi) * vehicle.groundspeed,
                   math.cos(vehicle.heading / 180.0 * math.pi) * vehicle.groundspeed)
    velocity_input = (pos_ctrl_dif[0] * K_p - K_d * current_vel[0], pos_ctrl_dif[1] * K_p - K_d * current_vel[1])
    graph_vel_input = (SCALE * math.cos((90 - heading_angle)/180.0*math.pi)*control_inputs[0]/5.0, SCALE * math.sin((90 - heading_angle)/180.0*math.pi)*control_inputs[0]/5.0)
    velocity_input = (velocity_input[0] + graph_vel_input[0], velocity_input[1] + graph_vel_input[1])
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_input[1], velocity_input[0], 0,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    '''msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading_angle,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        0,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)'''


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


def close_all():
    """
    Force the UAV to return to home and confirm to base that it reached such and can shut down...
    :return:
    """
    print('Return to launch')
    vehicle.mode = VehicleMode("RTL")
    while distance_to_current_waypoint() > 0.1:
        continue
    print("Close vehicle " + str(1) + " object")
    vehicle.close()
    s.send('Confirmed')
    s.close()
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()


class ListenToUpdates(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.start_sim = False
        self.shut_down = False
        self.ctrl_queue = dict()

    def run(self):
        while True:
            data_list = s.recv(BUFFER_SIZE)
            if data_list == "End!":
                self.shut_down = True
                break
            elif data_list == 'Go!':
                self.start_sim = True
                continue
            data_list = data_list.split()
            for i, dat in enumerate(data_list):
                data_list[i] = float(dat)
            self.ctrl_queue[data_list[0]] = data_list


class SendTrueLoc(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        while True:
            s.send('VehLoc ' + str(vehicle.location.global_frame.lat) + ' ' +
                   str(vehicle.location.global_frame.lon) + ' ' + str(vehicle.location.global_frame.alt) + ' ')


def execute_loop():
    """
    Arm and takeoff vehicle before establishing threads for constant communication with base and real-time control
    through use of the control_queue
    """

    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off
    # using a mission item (currently).
    arm_and_takeoff(10)  # TODO parameter associated with vehicle I.D.

    # Back and forth between UAV and base for confirming communication before starting simulation
    print('Start up thread used for listening to base')
    listener = ListenToUpdates()
    listener.start()
    print('Listening started!')
    while listener.start_sim is False:
        time.sleep(1)
        print('Waiting for confirmation from base to begin simulation...')
        continue
    s.send('Echoed Start!')
    print('Received go command from base and echoed back!')
    sender = SendTrueLoc()
    sender.start()
    print('Thread used for sending location to base started!')
    trans_count = 0

    # Wait until the queue is populated by the base...
    while listener.ctrl_queue == {}:
        continue

    # Begin real-time control loop
    real_time = 0.0
    real_time_start = time.time()
    while True:
        real_time_prev = real_time
        real_time = time.time() - real_time_start
        update_time = trans_count * TRANSITION_DURATION
        ctrl_loop(listener.ctrl_queue, update_time, real_time)
        time.sleep(0.01)
        if real_time_prev <= (trans_count + 1) * TRANSITION_DURATION <= real_time and real_time_prev != real_time:
            t_div = divmod(real_time, TRANSITION_DURATION)
            trans_count = int(round(t_div[0]))
            print('Updated the perceived control index in the queue')

        # TODO add in contigency plans and real-time checks here...

        # Check to see if listener received go-ahead to shut down the channel and return...
        if listener.shut_down is True:
            close_all()
            break


"""
script start
"""
sim_object = Sim_Object(segment_interval=SEGMENT_CONTROL_INTERVAL)
execute_loop()


