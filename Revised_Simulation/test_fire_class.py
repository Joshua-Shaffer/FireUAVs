from FireEnv import Env
import pygame
import time

window_size = [400, 400]
resolution = [1, 1]
obstacles_list = [[(45.0, 45.0), (40.0, 45.0), (40.0, 55.0), (45.0, 55.0)],
                  [(10.0, 20.0), (70.0, 20.0), (70.0, 10.0), (10.0, 10.0)],
                  [(45.0, 55.0), (55.0, 55.0), (55.0, 60.0), (45.0, 60.0)]]
domain = [[0.0, 100.0], [0.0, 100.0]]
domain1 = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
domain2 = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0]
starting_loc = [[50.1, 50.1]]
burn_length = 30.0
fire_env = Env(domain=domain, obstacle_list=obstacles_list)

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
        pygame.display.flip()
