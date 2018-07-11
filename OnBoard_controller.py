"""
DroneKit_simulator
Author: Joshua Shaffer (based on mission_basic.py example from DroneKit
Purpose: Test computer-based mission run of the fireUAV simulator in "real-time".
    Meant to figure out proper structure of running the simulator from a central
    node (i.e. the computer) and send proper commands to UAVs. Also aim to
    eventually test agency in UAVs (ability for them to override commands sent by
    the computer's simulation, but not commands sent by mission planner
"""

from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import pygame
#from FireUAV_modules import Sim_Object

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

time.sleep(1)

while True:
    # TODO: these are methods that the script must pass before executing
    # wait_receive_parameters()
    # confirm_origin_and_relative_pos()
    # any_other_checks()
    if not connection_string:
        import dronekit_sitl
        #sitl = dronekit_sitl.start_default()
        connection_string = 'tcp:127.0.0.1:57' + str(60 + (1) * 10)  # '127.0.0.1:145' + str(50 + (i)*10)
        print('Connecting to vehicle on: %s' % connection_strings[i])
        vehicle = connect(connection_string, wait_ready=True, baud=115200)
        origin = vehicle.location.global_frame
    break


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

def get_distance_metres_xy(aLocation1, aLocation2, scale):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return dlong * 1.113195e5 / scale, dlat * 1.113195e5 / scale

'''def distance_to_current_waypoint():
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
    return distancetopoint'''


'''def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()  # wait until download is complete.
    print(cmds)'''


def goto(dChange, tol_meter, scale):
    '''
    goto function for sending vehicle to relative waypoint, TODO revise scale
    '''

    check_statement = False
    targetLocation = list()
    for i, idx in enumerate(dChange):
        currentLocation=vehicle[i].location.global_frame
        targetLocation.append(get_location_metres(currentLocation, idx[1], idx[0]))
        vehicle[i].simple_goto(targetLocation[i], groundspeed=3.0)
        check_statement = check_statement or vehicle[i].mode.name=="GUIDED"

    while check_statement: #Stop action if we are no longer in guided mode.
        if update_visualizations(scale) is False:
            return False
        remainingDistance = list()
        for i, quad in enumerate(vehicle):
            print(quad.location.global_frame)
            remainingDistance.append(get_distance_metres(
                quad.location.global_frame, targetLocation[i]))
            print('Vehicle ' + str(i) + ' remaining distance: ' + str(remainingDistance[i]))
            print('Vehicle ' + str(i) + ' groundspeed is ' + str(quad.groundspeed))
        #remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        #print("Distance to target: ", remainingDistance)
        if all(dist<=tol_meter for dist in remainingDistance): #Just below target, in case of undershoot.
            print("All vehicles reached their targets")
            return True
        #time.sleep(2)
        check_statement = False
        for quad in vehicle:
            check_statement = check_statement or quad.mode.name == "GUIDED"


def send_vehicle(locs, scale, tol_meter):
    loc_rel = list()
    change_rel = list()
    for i, idx in enumerate(locs):
        loc_rel.append(get_distance_metres_xy(origin[0], vehicle[i].location.global_frame, scale))
        change_rel.append(((idx[0] - (loc_rel[i][0] + 1.0)) * scale,
                           (idx[1] - (loc_rel[i][1] + 1.0)) * scale))

    return goto(change_rel, tol_meter, scale)


def ctrl_loop(passed_time, start_loc, quad, K_p, K_d):
    '''
    Update loop for the real-time controller, tracks a reference (need to store instead
    of integrating each time...)
    :param passed_time:
    :param start_loc:
    :param quad:
    :param K_p:
    :param K_d:
    :return:
    '''
    Desired_state = sim_object.fleet.agents['UAV' + str(i + 1)].dynamic_model.integrate_state \
        (passed_time / 5.0, start_loc, sim_object.fleet.agents['UAV' + str(i + 1)].control_inputs,
         [0.0, 0.0, 0.0])
    heading_angle = 90 - Desired_state[2] * 180.0 / math.pi;
    Current_state = get_distance_metres_xy(origin[i], quad.location.global_frame, 1.0)
    pos_ctrl_dif = (Desired_state[0] * 10.0 - Current_state[0], Desired_state[1] * 10.0 - Current_state[1])
    current_vel = (math.sin(quad.heading / 180.0 * math.pi) * quad.groundspeed,
                   math.cos(quad.heading / 180.0 * math.pi) * quad.groundspeed)
    velocity_input = (pos_ctrl_dif[0] * K_p - K_d * current_vel[0], pos_ctrl_dif[1] * K_p - K_d * current_vel[1])
    msg = quad.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_input[1], velocity_input[0], 0,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    quad.send_mavlink(msg)
    '''msg = quad.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading_angle,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        0,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    quad.send_mavlink(msg)'''


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
    print('Return to launch')
    for i, quad in enumerate(vehicle):
        quad.mode = VehicleMode("RTL")
        print("Close vehicle " + str(i+1) + " object")
        quad.close()
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()


def execute_loop():
    """
    Start up the simulation and arm vehicles to starting altitude
    """

    print('Priming FireUAV simulation')

    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    arm_and_takeoff(10)  # TODO parameter associated with vehicle I.D.
    K_p = 1.0  # TODO: parameters associated with vehicle I.D.
    K_d = 0.1
    transition_duration = 5.0

    real_time_start = time.time()
    sim_time = 0.0
    sim_time_update = 1.0 # seconds
    sim_time_throttler = 0.1
    auto_run = True
    sim_update_number = 0

    while True:
        # if radio connection dropped, suspend and wait... (return home after so long)
        #listen()

        real_time = time.time() - real_time_start
        # if real_time % transition_duration = 0.0, push out current queue item and proceed forward



            start_loc = []
            for i, quad in enumerate(vehicle):
                heading = sim_object.fleet.agents['UAV' + str(i+1)].prev_state[2]
                value_pos = get_distance_metres_xy(origin[0], quad.location.global_frame, 1.0)
                start_loc.append([sim_object.fleet.agents['UAV' + str(i+1)].prev_state[0] - 1.0,
                                  sim_object.fleet.agents['UAV' + str(i + 1)].prev_state[1] - 1.0,
                                 heading])
            real_time_start = time.time()
            while time.time() - real_time_start <= (sim_object.params.update_step * 5.0):
                passed_time = time.time() - real_time_start
                for i, quad in enumerate(vehicle):
                    ctrl_loop(passed_time, start_loc[i], quad, K_p, K_d)

                if update_visualizations(10.0) is False:
                    close_all()
                    return
            #vehicle.mode = VehicleMode("AUTO")
            print('Updated step')

        sim_object.update(sim_time_throttler)

        sim_time = sim_time + sim_time_throttler
        #print(sim_time)

        #update()
        #transmit()


"""
script start
"""
execute_loop()


