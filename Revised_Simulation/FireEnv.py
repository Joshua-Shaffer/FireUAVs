'''
These two classes contain the environment as a whole and all individual cells in the environment. The update rules
regarding the cells still need to be added from Matlab code
'''
import math
from Fire_Model_Eqns import FireSimulator


class Env(object):
    def __init__(self, domain=[[5.0, 105.0], [5.0, 105.0]], starting_loc=[[50.1, 50.1]], burn_length=30.0, U=3.0, obstacle_list=[[]]):
        self.cells = {}
        self.fire_sim = FireSimulator(domain, starting_loc, burn_length, U=U)
        self.obstacles = obstacle_list
        self.fire_abstract_total = []
        self.domain = domain
        self.obs_loc_synth = []

    def add_cell(self, key, *args):
        self.cells[key] = Cell(*args)

    def update_cells(self):
        """
        Add/remove cells that are within/outside fire from the fire simulation
        -Serves as abstraction of the fire
        :return:
        """
        i = len(self.fire_sim.fire_list[0]) - 1
        new_total = list()
        #print(i)
        for cells in self.cells:
            #print(cells)
            #print(self.cells[cells].vertex_pts)
            #input('wait...')
            for pts in self.cells[cells].vertex_pts:
                #print(pts)
                for fires in self.fire_sim.fire_list:
                    if self.fire_sim.inside_poly(fires[i], pts):
                        new_total.append(pts)
                        break

        self.fire_abstract_total = new_total

    def update_abstractions_visuals(self, window_size, screen, pygame_instance, resolution):
        def transform_pt_to_pixels(ratio, domain, pts, window_size):
            new_pts = [int(round((pts[0]-domain[0][0])*ratio[0])),
                           window_size[1] - int(round((pts[1]-domain[1][0])*ratio[1]))]
            return new_pts

        m_to_pixel = (window_size[0]/(self.domain[0][1] - self.domain[0][0]),
                      window_size[1]/(self.domain[1][1] - self.domain[1][0]))
        WIDTH = int(round(float(window_size[0]) / (self.domain[0][1] - self.domain[0][0]) * resolution[0]))
        HEIGHT = int(round(float(window_size[1]) / (self.domain[1][1] - self.domain[1][0]) * resolution[1]))

        for pts in self.fire_abstract_total:
            #print(transform_pt_to_pixels(m_to_pixel, self.domain, pts,
            #                             window_size))
            pygame_instance.draw.circle(screen, (23, 23, 12),
                                transform_pt_to_pixels(m_to_pixel, self.domain, pts,
                                         window_size), 2)


class Cell(object):
    def __init__(self, loc, number_sub_points, dim):
        self.loc = loc
        self.dim = dim

        locations = list()
        for idx, d in enumerate(dim):
            length_side = d/number_sub_points
            ind_loc = list()
            for i in range(0, number_sub_points):
                if i == 0:
                    ind_loc.append(loc[idx]+length_side/2.0)
                elif i < number_sub_points:
                    ind_loc.append(ind_loc[i-1]+length_side)
                elif i == number_sub_points:
                    ind_loc.append(ind_loc[i-1]+length_side)
            locations.append(ind_loc)

        vertex_pts = list()
        # Only gonna support 2 dimensions atm
        for locs1 in locations[0]:
            for locs2 in locations[1]:
                vertex_pts.append((locs1, locs2))
        self.vertex_pts = vertex_pts

    def __str__(self):
        return 'Need to update this for Cell'

    def __repr__(self):
        return self.__str__()
