from FireEnv import Env
import pygame
import time
import math

window_size = [400, 400]
resolution = [1, 1]
#obstacles_list = [[(45.0, 45.0), (40.0, 45.0), (40.0, 55.0), (45.0, 55.0)],
#                  [(10.0, 20.0), (70.0, 20.0), (70.0, 10.0), (10.0, 10.0)],
#                  [(45.0, 55.0), (55.0, 55.0), (55.0, 60.0), (45.0, 60.0)]]
domain = [[0.0, 100.0], [0.0, 100.0]]
domain1 = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
domain2 = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
starting_loc = [[50.1, 50.1]]
burn_length = 30.0
fire_env = Env(domain=domain)#, obstacle_list=obstacles_list)

for x in domain1:
    for y in domain2:
        fire_env.add_cell((x, y), [x, y], 2, [10.0, 10.0])

#fire_sim = FireSimulator(domain, starting_loc, burn_length, U=1.0)

# Initialize pygame for displaying simulation
pygame.init()
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption("Fire Simulation")
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
closer = False
'''
for mins in range(0, 1000):
    dt = mins
    time.sleep(0.10)
    print(mins)
    if fire_env.fire_sim.update_simulation(float(dt), obstacles_list) is True:
        print('That\'s all folks')
        break
    fire_env.update_cells()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            closer = True

    if closer is True: break

    if divmod(dt, 10)[1] == 0:
        pygame.draw.rect(screen, (6, 132, 46), [0, 0, window_size[0], window_size[1]])
        #print(fire_sim.fire_list)
        fire_env.fire_sim.update_fire_visuals(window_size, screen, pygame, resolution, obstacles_list)
        fire_env.update_abstractions_visuals(window_size, screen, pygame, resolution)

        # Insert visualization update here
        # Limit to 60 frames per second
        clock.tick(20)

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()'''


def drop_suppresant_loc(loc_c, loc, fire_current, obstacles, suppressant_obstacles, duration, env, time):
    """
    Method to create a 'fill-in' obstacle for when a UAV drops suppressant.
    :param loc:
    :param fire_current:
    :param obstacles:
    :param suppressant_obstacles:
    :param duration:
    :param env:
    :return:
    """
    if loc_c in suppressant_obstacles:
        if duration+time > suppressant_obstacles[loc_c][1][0]+suppressant_obstacles[loc_c][1][1]:
            suppressant_obstacles[loc_c][1] = (duration, time)
            return

    polygon_start = loc
    for idx, fires_hist in enumerate(fire_current):
        #print(fires_hist)
        fires = fires_hist[len(fires_hist)-1]
        #print(fires)
        inside_vertexes = list()
        polygon_temp = list()
        fire_lines_and_intersections = list()
        for idx2, pts in enumerate(polygon_start):
            polygon_temp.append(pts)
            poly_line = [pts, polygon_start[idx2 + 1]] if idx2 + 1 < len(polygon_start) else [pts, polygon_start[0]]
            pt_list = list()
            for idx3, fire_vertex in enumerate(fires):
                fire_line = [fire_vertex, fires[idx3 + 1]] if idx3 + 1 < len(fires) else [fire_vertex, fires[0]]
                if env.fire_sim.intersect(poly_line[0], poly_line[1], fire_line[0], fire_line[1]):
                    print('poly line and fire line')
                    print(poly_line)
                    print(fire_line)
                    in_pt = [env.fire_sim.intersection_point(poly_line, fire_line)[0],
                    env.fire_sim.intersection_point(poly_line, fire_line)[1]]
                    pt_list.append((in_pt, fire_line))
            pt_list.sort(key=lambda p: (p[0][0] - pts[0]) ** 2 + (p[0][1] - pts[1]) ** 2)
            for extra in pt_list:
                polygon_temp.append(extra[0])
                fire_lines_and_intersections.append(extra)

        fire_vertex_inside = list()
        for idx2, pts in enumerate(fires):
            if env.fire_sim.inside_poly(polygon_start, pts):
                fire_vertex_inside.append(pts)

        polygon_temp2 = list()
        print('Polygon_temp')
        print(polygon_temp)
        for idx2, pts in enumerate(polygon_temp):
            if idx2 == 0:
                idx_before = len(polygon_temp) - 1
                idx_after = idx2 + 1
            elif idx2 == len(polygon_temp) - 1:
                idx_before = idx2 - 1
                idx_after = 0
            else:
                idx_before = idx2 - 1
                idx_after = idx2 + 1
            print(idx2)
            print(pts)
            print(polygon_temp[idx_before])
            print(polygon_temp[idx_after])

            print('check conditions')
            print(env.fire_sim.inside_poly(fires, pts))
            print(env.fire_sim.inside_poly(fires, polygon_temp[idx_before]))
            print(env.fire_sim.inside_poly(fires, polygon_temp[idx_after]))

            if (not env.fire_sim.inside_poly(fires, pts) or not env.fire_sim.inside_poly(fires,
                                                                                         polygon_temp[idx_before])
                    or not env.fire_sim.inside_poly(fires, polygon_temp[idx_after])):
                polygon_temp2.append(pts)

        polygon_temp3 = list()
        print('Polygon_temp2')
        print(polygon_temp2)
        print('Fire lines and intersections')
        print(fire_lines_and_intersections)
        for idx2, pts in enumerate(polygon_temp2):
            next_idx = idx2 + 1 if idx2 < len(polygon_temp2) - 1 else 0
            polygon_temp3.append(pts)
            if env.fire_sim.inside_poly(fires, pts) and env.fire_sim.inside_poly(fires, polygon_temp2[next_idx]):
                direction_for = True
                for idx3, pts2 in enumerate(fire_lines_and_intersections):
                    print(pts2)
                    dif_mag = math.sqrt(\
                        (pts2[0][0] - pts[0]) ** 2 + (pts2[0][1] - pts[1]) ** 2)
                    if dif_mag < 0.000001:
                        print(pts)
                        print(idx2)
                        print(idx3)
                        if pts2[1][0] == fire_vertex_inside[0] or pts2[1][1] == fire_vertex_inside[0]:
                            direction_for = True
                            break
                        elif pts2[1][0] == fire_vertex_inside[len(fire_vertex_inside) - 1] or \
                                pts2[1][1] == fire_vertex_inside[len(fire_vertex_inside) - 1]:
                            direction_for = False
                            break
                if direction_for == True:
                    for ii in fire_vertex_inside:
                        if ii not in polygon_temp2: polygon_temp3.append(ii)
                else:
                    for ii in reversed(fire_vertex_inside):
                        if ii not in polygon_temp2: polygon_temp3.append(ii)
        polygon_start = polygon_temp3


    suppressant_obstacles.update({loc_c: [polygon_start, (duration, time)]})
    obstacles.append(polygon_start)


fire_poly = [[[[45.0, 40.0], [65.1, 40.0], [60.1, 60.0], [58.5, 55.5], [50.1, 50.0]]]]
fire_env.fire_sim.fire_list = fire_poly
loc_end = [[50.0, 50.0], [50.0, 60.0], [60.0,60.0], [60.0, 50.0]]
loc_end2 = list()
for items in reversed(loc_end):
    loc_end2.append(items)
print(loc_end2)
loc_end = loc_end2

pygame.draw.rect(screen, (6, 132, 46), [0, 0, window_size[0], window_size[1]])
# print(fire_sim.fire_list)
fire_env.fire_sim.update_fire_visuals(window_size, screen, pygame, resolution, [])

obstacles_list = []
suprresss = {}
drop_suppresant_loc((50, 50), loc_end, fire_poly, obstacles_list, suprresss, 20.0, fire_env, 0.0)
print(obstacles_list)
pygame.draw.rect(screen, (6, 132, 46), [0, 0, window_size[0], window_size[1]])
fire_env.fire_sim.update_fire_visuals(window_size, screen, pygame, resolution, obstacles_list)
# Insert visualization update here
closer = True
while closer:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            closer = False
    clock.tick(20)

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

