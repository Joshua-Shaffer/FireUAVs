import math
import numpy as np

def allocation_function(fleet, params, env, time):

    fireLocs = list()
    for i in env.cells_in_abstract_total:
        fireLocs.append(i)#fireLocs.append((pts[0]/10.0,pts[1]/10.0))

    locs = list()
    goals = list()
    for i in fleet.agents:
        locs.append(fleet.agents[i].state_belief)
        goals.append(fleet.agents[i].goal)

    if fireLocs == []:
        return

    no_agents = len(locs)

    UAVs = np.zeros((no_agents, 3))

    j = 0
    for loc in locs:
        UAVs[j][0] = loc[0]
        UAVs[j][1] = loc[1]
        UAVs[j][2] = j
        j+=1

    num_UAVs = len(UAVs)

    # Wind angle rotation matrix
    wind_angle = env.u_angle/180.0*math.pi
    rot_w = [[math.cos(wind_angle), math.sin(wind_angle)],[-math.sin(wind_angle), math.cos(wind_angle)]]
    # Assign UAVs to fires in cluster centroid_id
    uav_total_costs = list()

    for l in range(num_UAVs):
        uav_fire_costs = list()
        #print(fireLocs)
        for k in range(len(fireLocs)):

            fire_idx = k
            assigned_fire = fireLocs[fire_idx]
            x1 = float(UAVs[l][0])
            x2 = float(assigned_fire[0])
            y1 = float(UAVs[l][1])
            y2 = float(assigned_fire[1])
            #print(x2)
            #print(y2)
            #input('wait..')
            if (int(round(x2)), int(round(y2))) == (10, 10) or (int(round(x2)), int(round(y2))) == (10, 1) or \
                (int(round(x2)), int(round(y2))) == (1, 1) or (int(round(x2)), int(round(y2))) == (1, 10):
                continue

            # Find fire with least cost for UAV to go to
            distance_fire_UAV  = (x1 - x2) ** 2 + (y1 - y2) ** 2
            fire_edges = np.array([x2, y2, params.width - x2, params.height - y2])
            fire_edge_closest = np.min(fire_edges)
            wind_pos_x = env.fire_sim.wind_speed*(rot_w[0][0]*x2+rot_w[0][1]*y2)
            if (str(int(round(x2))), str(int(round(y2)))) in env.suppressant_obstacles:
                #print(time)
                #print(env.suppressant_obstacles[(str(int(round(x2))), str(int(round(y2))))][1][1])
                #print(env.suppressant_obstacles[(str(int(round(x2))), str(int(round(y2))))][1][0])
                burn_through_time = - time + \
                                    env.suppressant_obstacles[(str(int(round(x2))), str(int(round(y2))))][1][1] + \
                                    env.suppressant_obstacles[(str(int(round(x2))), str(int(round(y2))))][1][0]
            #    print(env.suppressant_obstacles[(str(int(round(x2))), str(int(round(y2))))])
            else:
                burn_through_time = 0.0
            if burn_through_time < 0.0:
                burn_through_time = 0.0
            #print(burn_through_time)
            cost = params.Dc_coeff*distance_fire_UAV + params.Ef* \
                fire_edge_closest - params.Wf*wind_pos_x + params.Bf*burn_through_time
            #print('Weighting Terms')
            #print('Fire: (' + str(x2) + ', ' + str(y2) + ') | Dc: ' + str(params.Dc_coeff*distance_fire_UAV) + ' | '
            #      + ' | Ef: ' + str(params.Ef*fire_edge_closest) +
            #      ' | Wf: ' + str(params.Wf*wind_pos_x) + ' | Bf: ' + str(params.Bf*burn_through_time))
            uav_fire_costs.append([cost, k, fireLocs[fire_idx]])

        uav_fire_costs.sort(key=lambda p: p[0])
        uav_total_costs.append(uav_fire_costs)

    uavs_to_water = list()
    for l in range(num_UAVs):
        UAV_idx = int(UAVs[l][2])
        UAV_name = 'UAV' + str(UAV_idx+1)

        if fleet.agents[UAV_name].water_level == 0:
            #print('UAV ' + UAV_name + ' SENT TO BASE')
            fleet.agents[UAV_name].update_objective_state(params.base_location)
            fleet.agents[UAV_name].sync_signal = 1
            uavs_to_water.append(l)
            continue

    assigned_fires = list()
    #print(uav_total_costs)
    for l in range(num_UAVs):
        UAV_idx = int(UAVs[l][2])
        UAV_name = 'UAV' + str(UAV_idx+1)
        if l in uavs_to_water: continue

        li = 0
        while True:
            fire = uav_total_costs[l][li][1]
            if fire in assigned_fires:
                li = li + 1
                if li != len(uav_total_costs[l]):
                    continue
                if li == len(uav_total_costs[l]):
                    li = 0

            fire_min = fireLocs[fire]
            assigned_fires.append(fire)
            assign_pts = [float(fire_min[0]), float(fire_min[1])]
            if assign_pts[0] > 10.0 or assign_pts[0] < 1.0 or assign_pts[1] > 10.0 or assign_pts[1] < 1.0:
                assign_pts = [3.0, 3.0]
            fleet.agents[UAV_name].update_objective_state(
                assign_pts)  # fleet.agents[UAV_name].update_objective_state(fire_min)
            fleet.agents[UAV_name].sync_signal = 1
            break



