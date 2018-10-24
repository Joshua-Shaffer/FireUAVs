"""
OnBoard_controller.py
Author: Joshua Shaffer
Date: July 23, 2018
Purpose: Drone-kit python script that communicates with control commands received from the ground.
        Implements the dynamic graph constructed with a feedback/feedforward control loop
"""

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
import time
import math
from pymavlink import mavutil
from Dynamics import Dynamics
from Graph import Graph

#  Set up option parsing to get connection string and the associated vehicle ID used
#  Used for testing in a terminal, needs to be initialized from bash script with appropriate arguments
#parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
#parser.add_argument('--connect',
                   #help="vehicle connection target string. If not specified, SITL automatically started and used.")
#parser.add_argument('--vehicle_id',help="ID of vehicle that this script is tied to...")
#args = parser.parse_args()

#connection_string = args.connect
#vehicle_id = args.vehicle_id
#sitl = None


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


def ctrl_loop(single_key_list, ctrl_queue, timer, real_time, ctrl_list):
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
    start_key = ctrl_queue[timer/TRANSITION_DURATION][1]
    next_key = ctrl_queue[timer/TRANSITION_DURATION][3]

    if passed_time/TRANSITION_DURATION > 1.0:
        time_input = SEGMENT_CONTROL_INTERVAL

    control_inputs = graph.graph[single_key_list[start_key]] \
        .children[single_key_list[next_key]][0]

    desired_state = graph.graph[single_key_list[start_key]].children \
        [single_key_list[next_key]][2][round(SEGMENT_CONTROL_INTERVAL*(round(time_input)/SEGMENT_CONTROL_INTERVAL))/SEGMENT_CONTROL_INTERVAL]
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

    if heading_angle < 0:
        heading_angle = 360 + heading_angle
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading_angle,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        0,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


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
    #s.send('Confirmed')
    #s.close()
    # Shut down simulator if it was started.
    #if sitl is not None:
    #    sitl.stop()

#class SendTrueLoc(Thread):
#    def __init__(self):
#        Thread.__init__(self)
#
#    def run(self):
#        while True:
#            #print(vehicle.heading)
#            s.send('VehLoc ' + str(vehicle.location.global_frame.lat) + ' ' +
#                   str(vehicle.location.global_frame.lon) + ' ' + str(vehicle.location.global_frame.alt) + ' '
#                   + str(vehicle.heading) + ' ')


def execute_loop():
    """
    Arm and takeoff vehicle before establishing threads for constant communication with base and real-time control
    through use of the control_queue
    """
    trans_count = 0

    # Begin real-time control loop
    real_time = 0.0
    real_time_start = time.time()
    while True:
        if vehicle.parameters['TIME_DEFAULT'] not in ctrl_queue:
            ctrl_queue[vehicle.parameters['TIME_DEFAULT']] = [vehicle.parameters['GRAPH_DEFAULT'],
                                                  vehicle.parameters['CTRL_DEFAULT']]
        real_time_prev = real_time
        real_time = time.time() - real_time_start
        update_time = trans_count * TRANSITION_DURATION
        ctrl_loop(ctrl_queue, update_time, real_time)
        time.sleep(0.05)
        if real_time_prev <= (trans_count + 1) * TRANSITION_DURATION <= real_time and real_time_prev != real_time:
            t_div = divmod(real_time, TRANSITION_DURATION)
            trans_count = int(round(t_div[0]))
            print('Updated the perceived control index in the queue')

        # Check to see if listener received go-ahead to shut down the channel and return...
        if vehicle.parameters['PARAMETER_DKM'] == 10:
            close_all()
            break


"""
script start
"""
"""''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''"""
'''INITIALIZATION and PARAMETERS'''
# Temporary, should modify to work on native build of SITL if we wish to test multiple vehicles

connection_string = 'tcp:127.0.0.1:57' + str(60 + 1 * 10)  # '127.0.0.1:145' + str(50 + (i)*10)
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=115200)

vehicle.parameters['PARAMETER_DKM']=100
vehicle.flush()
mission_pass1_statement = False
while mission_pass1_statement == False:
    pass1 = (vehicle.parameters['SWARM_ID'] != 0)
    pass2 = (vehicle.parameters['ORILAT'] != 0.0)
    pass3 = (vehicle.parameters['ORILONG'] != 0.0)
    pass4 = (vehicle.parameters['ORIALT'] != 0.0)
    pass5 = (vehicle.parameters['VEL_K_GAIN'] != -10000.0)
    pass6 = (vehicle.parameters['POS_K_GAIN'] != -10000.0)
    pass7 = (vehicle.parameters['SCALE_GRAPH'] != 0.0)
    pass8 = (vehicle.parameters['TRANS_DUR'] != 0.0)
    mission_pass1_statement = pass1 and pass2 and pass3 and pass4 and pass5 and pass6 and pass7 and pass8
    time.sleep(2)

origin = LocationGlobal(vehicle.parameters['ORILAT'], vehicle.parameters['ORILONG'], vehicle.parameters['ORIALT'])
K_p = vehicle.parameters['POS_K_GAIN']
K_d = vehicle.parameters['VEL_K_GAIN']
TRANSITION_DURATION = vehicle.parameters['TRANS_DUR']
SCALE = vehicle.parameters['SCALE_GRAPH']
SEGMENT_CONTROL_INTERVAL = 100
ctrl_queue = dict()

graph = Graph(Dynamics)
graph.generate_graph_from_dynamics([1.0, 1.0, math.pi / 2.0],
                                      [3, (1., 10., 0), (1., 10., 0), (0., 3.0 * math.pi / 2.0, 1)],
                                      [[1., 0.], [1.0 * math.pi / 2.0, math.pi / 2.0],
                                       [1.0 * math.pi / 2.0, -math.pi / 2.0],
                                       [0.0, 0.0]], 1., 0.1, 0.1, SEGMENT_CONTROL_INTERVAL)
for nodes in graph.graph:
    graph.key_to_nodes.append(nodes)
"""'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''"""
'''BEGIN ARM AND TAKE OFF, THEN LET GCS KNOW YOU'RE READY'''
arm_and_takeoff(7 + vehicle.parameters['SWARMID']*4)
vehicle.parameters['PARAMETER_DKM']=1
vehicle.flush()

"""'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''"""
'''Wait for GCS to give the execute command'''
while vehicle.parameters['PARAM_DKM']!=2:
    continue
"""'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''"""

execute_loop()


