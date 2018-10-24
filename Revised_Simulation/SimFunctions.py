'''
Primary simulation loop and interpreter functions for identifying where a UAV is in relation to synthesized
controllers (will want to move the latter to different module)
'''
import pygame
import math
import time


# These following two functions are HIGHLY dependent on the format of state for this simulation and the files they call


def simulation_loop(fleet, env, Params, visualize=False):

    # Setup visuals
    visualize2 = False
    if visualize:
        pygame.init()
        window_size = [(Params.WIDTH * Params.width),
               (Params.HEIGHT) * Params.height]
        #print(window_size)
        #input('wait...')
        screen = pygame.display.set_mode(window_size)
        pygame.display.set_caption("UAVs on Fire")
        # Used to manage how fast the screen updates
        clock = pygame.time.Clock()
        visualize2 = True

    t = 0.0
    continue_sim = False
    time_start = time.time()
    time_start2 = time.time()
    while t <= Params.sim_time:

        '''VISUALIZATION FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'''
        if visualize:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    continue_sim = not continue_sim
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_1:
                    visualize2 = not visualize2

        if visualize2:

            for row in range(Params.height):
                for column in range(Params.width):
                    color = Params.fuel_color[6]

                    pygame.draw.rect(screen, color,
                             [(Params.WIDTH) * column,
                              (Params.HEIGHT) * (row), Params.WIDTH,
                              Params.HEIGHT])
                    pygame.draw.rect(screen, (0, 0, 0),
                             [(Params.WIDTH) * column,
                              (Params.HEIGHT) * (row), Params.WIDTH,
                              Params.HEIGHT], Params.MARGIN_HALF)

            env.fire_sim.update_obstacle_visuals(window_size, screen, pygame, [1, 1], env.obstacles)
            env.update_suppressant_visuals(window_size, screen, pygame)
            env.fire_sim.update_fire_visuals(window_size, screen, pygame, [1, 1], env.obstacles)
            env.update_abstractions_visuals(window_size, screen, pygame, [1, 1])

            for i in fleet.agents:
                locs_use = fleet.agents[i].display_loc(Params)
                pygame.draw.polygon(screen, (94, 154, 249), locs_use[0])
                pygame.draw.line(screen, (0,0,0), locs_use[1][0], locs_use[1][1], 2)

            # Insert visualization update here
            # Limit to 60 frames per second
            clock.tick(20)

            # Go ahead and update the screen with what we've drawn.
            pygame.display.flip()
        '''<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<'''

        '''SIMULATION FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>'''
        if continue_sim:
            if abs(divmod(t, 15.0)[1]) < 0.0001 or abs(divmod(t, 15.0)[1] - 15.0) < 0.0001:
                env.fire_sim.update_simulation(15.0/1.0, env.obstacles, env.suppressant_obstacles, t, Params)
                #print('Fire sim update: ' + str(time_start - time.time()))
                time_start = time.time()
                env.update_cells()
                #print('Cell abstr update: ' + str(time_start - time.time()))
                time_start = time.time()
                for hist in env.fire_sim.fire_list:
                    #print('Total fire size: ' + str(len(hist[len(hist)-1])) + ' vertices')
                    for idx,pts in enumerate(hist[len(hist)-1]):
                        if pts[0] < env.domain[0][0] or pts[0] > env.domain[0][1] or pts[1] < env.domain[1][0] \
                         or pts[1] > env.domain[1][1]:
                            print(t)
                            print('Total runtime: ' + str(time.time() - time_start2))
                            return t
                #print('Check edge: ' + str(time_start - time.time()))
                #time_start = time.time()

            tar = divmod(t, Params.update_step)
            if math.fabs(tar[1]) < 0.1 * Params.sim_throttle or tar[1] == 0.0 or \
                    math.fabs(tar[1] - Params.update_step) < 0.1 * Params.sim_throttle:
                print(t)
                fleet.allocate(env, Params, t)
                #print('Allocate: ' + str(time_start - time.time()))
                #time_start = time.time()
                fleet.update_ctrls(env, t, Params)
                #print('Update_ctrls: ' + str(time_start - time.time()))
                #time_start = time.time()

            tar = divmod(t, Params.time_step)
            #print(t)
            #print(Params.time_step)
            #print(tar)
            #time_start = time.time()
            if math.fabs(tar[1]) >= 0.0001 and tar[1] != 0.0 and math.fabs(tar[1] - Params.time_step) >= 0.0001:
                asdf = 1
                #input('SKIPPED SOME INTEGRATION>>>')
            if math.fabs(tar[1]) < 0.0001 or tar[1] == 0.0 or math.fabs(tar[1] - Params.time_step) < 0.0001:
                fleet.update(env, Params, Params.time_step)
                #print('Update UAV: ' + str(time_start - time.time()))
                #time_start = time.time()
                env.update_suppressant_obstacles(t)
                #print('Real update: ' + str(time_start - time.time()))
                #time_start = time.time()

            t = t + Params.sim_throttle
        '''<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<'''

    return 'No results yet bud'