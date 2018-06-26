import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

def allocation_function(fleet, params, env):
    fires = list()
    fireLocs = list()
    for i in env.cells:
        if env.cells[i].fire > 0:
            fires.append([i, env.cells[i].fire])
            fireLocs.append(i)

    locs = list()
    goals = list()
    for i in fleet.agents:
        locs.append(fleet.agents[i].state_belief)
        goals.append(fleet.agents[i].goal)

    Nassign = 0
    no_agents = len(locs)
    #print(fires)
    if fires == []:
        return

    maxk = math.ceil((len(fires)+1)/2) # Or maxk = len(fires), if more clusters are desired
    numClusters, centroids, F = generate_clusters(fireLocs, int(maxk))

    # Compute cluster priorities and associate them with their cluster id
    C = np.zeros((numClusters, 3))

    for i in range(len(F)):
        cluster_id = F[i]
        fire_intensity = fires[i][1]
        C[cluster_id][0] = cluster_id
        C[cluster_id][1] = C[cluster_id][1] + fire_intensity
        

    C = C[C[:,1].argsort()][::-1]
    tot_priority = sum(C[:,1])

    Nassign = 0
    start = 0

    no_agents = len(locs)
    no_total = 0
    tol = 0.00000001
    for i in range(numClusters):

        if (no_agents - Nassign > 0):
            num_UAV_assigned = math.ceil((float(C[i][1])/float(tot_priority))*(no_agents))
        else:
            num_UAV_assigned = 0

        C[i][2] = num_UAV_assigned
        Nassign = Nassign + num_UAV_assigned
        
        centroid_id = int(C[i][0])

        #print centroids 
        cx = centroids[centroid_id][0]
        cy = centroids[centroid_id][1]

        UAVs = np.zeros((no_agents, 4))
        
        j = 0
        for loc in locs:
            distance_centroid_UAV = (cx - loc[0]) ** 2 + (cy - loc[1]) ** 2
            benefit = params.Cs_coeff*(C[i][1]) - params.Dc_coeff*distance_centroid_UAV
            UAVs[j][0] = benefit
            UAVs[j][1] = loc[0]
            UAVs[j][2] = loc[1]
            UAVs[j][3] = j
            j+=1

        UAVs = UAVs[UAVs[:,0].argsort()][::-1]

        # UAVS assigned to cluster centroid_id
        max_idx = int(num_UAV_assigned + no_total);
        if (max_idx > no_agents):
            max_idx = int(no_agents - 1)
        
        assigned_UAVs = UAVs[start:max_idx]
        start = max_idx 
        no_total = num_UAV_assigned + no_total 
        
        fires_selected = list()
        num_UAVs = len(assigned_UAVs)
        assigned_fireLocs = np.where(F == centroid_id)[0]
        #print(assigned_fireLocs)

        # Assign UAVs to fires in cluster centroid_id
        for l in range(num_UAVs):
            UAV_idx = int(assigned_UAVs[l][3])
            UAV_name = 'UAV' + str(UAV_idx+1)

            if fleet.agents[UAV_name].water_level == 0:
                fleet.agents[UAV_name].update_objective_state(params.base_location)
                fleet.agents[UAV_name].sync_signal = 1
                continue

            min_cost = 10000

            for k in range(len(assigned_fireLocs)):
                 
                fire_idx = assigned_fireLocs[k]
                assigned_fire = fireLocs[fire_idx]
                UAV_idx = int(assigned_UAVs[l][3])
                x1 = float(assigned_UAVs[l][1])
                x2 = float(assigned_fire[0])
                y1 = float(assigned_UAVs[l][2])
                y2 = float(assigned_fire[1])

                # IF UAV is not already at the fire
                if (abs(x1-x2) > tol or abs(y1-y2) > tol):
                    # Find fire with least cost for UAV to go to
                    distance_fire_UAV  = (x1 - x2) ** 2 + (y1 - y2) ** 2
                    distance_cluster_fire = (x2 - cx) ** 2 + (y2 - cy) ** 2
                    fire_intensity = fires[fire_idx][1]
                    cost = params.Dc_coeff*distance_fire_UAV - params.Cs_coeff*(fire_intensity + distance_cluster_fire)
                    
                    if goals[UAV_idx]:
                        gx = goals[UAV_idx][0]
                        gy = goals[UAV_idx][1]
                        distance_to_goal = (x2-gx) ** 2 + (y2-gy) ** 2
                        cost = cost + params.switch_penalty*distance_to_goal

                    #print('if state 1')
                    if (min_cost > cost):
                        min_cost = cost
                        chosen_fire = fire_idx
                        fire_ind = k
                        #print(fire_ind)
                else:
                    #print('if state 2')
                    if (len(assigned_fireLocs) < 2):
                        #print('if state 3')
                        chosen_fire = fire_idx
                        fire_ind = k
            
            fires_selected.append(fireLocs[chosen_fire]) 
            
            if params.mode_version is 'Sync':
                # Assign Multiple UAVs to the fire with least cost
                fire_min = fires_selected[0]
                fleet.agents[UAV_name].update_objective_state(fire_min)
                if (num_UAVs > 1):
                    fleet.agents[UAV_name].sync_signal = 1
                else:
                    fleet.agents[UAV_name].sync_signal = 1  # TODO will be changed eventually
            else:
                if len(assigned_fireLocs)>0:
                    fleet.agents[UAV_name].update_objective_state(fireLocs[chosen_fire])
                    fleet.agents[UAV_name].sync_signal = 1  # TODO will be changed eventually
                    # Remove fire from array
                    #print(fire_ind)
                    assigned_fireLocs = np.delete(assigned_fireLocs, fire_ind, 0)


def generate_clusters(fireLocs, maxK):

    # This function finds the optimum k value for performing a a k-means clustering algorithm.  
    # Then it returns the number of clusters (Nc), list of clusters centroids (C) and fire labels. 

    k_values = np.zeros((maxK, 1))
    SSE_results = np.zeros((maxK, 2))
    X = np.array(fireLocs)

    # Find the optimal k by calculating the sum squared error (SSE)
    for k in range(maxK):
        kmeans = KMeans(k+1).fit(X)
        labels = kmeans.labels_
        cluster_centers = kmeans.cluster_centers_

        for i in range(len(X)):
            idx = labels[i]
            fx = X[i][0]
            fy = X[i][1]
            cx = cluster_centers[idx][0]
            cy = cluster_centers[idx][1]
            SSE = ((fx - cx) ** 2 + (fy - cy) ** 2)
            k_values[idx] = SSE + k_values[idx]

        SSE_results[k][0] = k
        SSE_results[k][1] = np.average(k_values[0:k+1])
        k_values = np.zeros((maxK, 1))

    # Elbow Method to find optimal k
    if (len(SSE_results[:,1])) > 2:
        secondDerivative = np.diff(SSE_results[:,1], n=2)
        idxNc = np.where(secondDerivative == max(secondDerivative))[0]
        Nc = idxNc[0] + 3
    else:
        idxNc = np.where(SSE_results[:,1] == min(SSE_results[:,1]))[0]
        Nc = idxNc[0] + 1

    # plt.plot(SSE_results[:,0], SSE_results[:,1])
    # plt.show()
     
    kmeans = KMeans(Nc, random_state=0).fit(X)

    return [Nc, kmeans.cluster_centers_, kmeans.labels_]



