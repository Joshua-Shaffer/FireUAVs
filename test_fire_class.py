from Fire_Model_Eqns import FireSimulator
import pygame
import time

window_size = [400, 400]
resolution = [1, 1]
obstacles_list = [[(45.0, 45.0), (40.0, 45.0), (40.0, 55.0), (45.0, 55.0)],
                  [(10.0, 20.0), (70.0, 20.0), (70.0, 10.0), (10.0, 10.0)],
                  [(45.0, 55.0), (55.0, 55.0), (55.0, 60.0), (45.0, 60.0)]]
domain = [[0.0, 100.0], [0.0, 100.0]]
starting_loc = [[50.1, 50.1]]
burn_length = 30.0
fire_sim = FireSimulator(domain, starting_loc, burn_length, U=10.0)

# Initialize pygame for displaying simulation
pygame.init()
screen = pygame.display.set_mode(window_size)
pygame.display.set_caption("Fire Simulation")
# Used to manage how fast the screen updates
clock = pygame.time.Clock()
closer = False

for mins in range(0, 1000):
    dt = mins
    time.sleep(0.01)
    print(mins)
    if fire_sim.update_simulation(float(dt), obstacles_list) is True:
        print('That\'s all folks')
        break

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            closer = True

    if closer is True: break

    if divmod(dt, 10)[1] == 0:
        pygame.draw.rect(screen, (6, 132, 46), [0, 0, window_size[0], window_size[1]])
        #print(fire_sim.fire_list)
        fire_sim.update_fire_visuals(window_size, screen, pygame, float(60.0), resolution, obstacles_list)

        # Insert visualization update here
        # Limit to 60 frames per second
        clock.tick(20)

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()
