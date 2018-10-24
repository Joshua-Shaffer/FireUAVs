'''
These two classes contain the environment as a whole and all individual cells in the environment. The update rules
regarding the cells still need to be added from Matlab code
'''
import math
from Fire_Model_Eqns import FireSimulator


class Env(object):
    def __init__(self, domain=[[5.0, 105.0], [5.0, 105.0]], starting_loc=[[25.1, 45.1], [45.1, 25.1]], burn_length=30.0, U=6.0, obstacle_list=[[]],
                 U_angle=0.0, R=1.0):
        self.cells = {}
        self.fire_sim = FireSimulator(domain, starting_loc, burn_length, U=U, R=R, theta=U_angle)
        self.obstacles = obstacle_list
        self.fire_abstract_total = []
        self.domain = domain
        self.obs_loc_synth = []
        self.suppressant_obstacles = {}
        self.u_angle = U_angle
        self.R = R
        self.cells_in_abstract_total = []
        self.cells_with_abstractions = []
        self.obs_abstraction = []

    def add_cell(self, key, *args):
        self.cells[key] = Cell(*args)
        self.cells_with_abstractions.append(key)

    def update_cells(self):
        """
        Add/remove cells that are within/outside fire from the fire simulation
        -Serves as abstraction of the fire
        :return:
        """
        i = len(self.fire_sim.fire_list[0]) - 1
        new_total = list()
        self.cells_in_abstract_total = []
        cells_abstracted = []
        for cells in self.cells:
            if cells in self.obs_abstraction:
                continue
            trip = False
            for xtra in range(-1, 2):
                for ytra in range(-1, 2):
                    if (cells[0] + xtra, cells[1] + ytra) in self.cells_with_abstractions:
                        trip = True
                    if trip is True:
                        break
                if trip is True:
                    break
            if trip is False:
                continue

            pts_in_fire = list()
            for pts in self.cells[cells].vertex_pts:
                if pts in self.fire_abstract_total:
                    new_total.append(pts)
                    pts_in_fire.append(pts)
                    continue
                for fires in self.fire_sim.fire_list:
                    if self.fire_sim.inside_poly(fires[i], pts):
                        new_total.append(pts)
                        pts_in_fire.append(pts)
                        break
            if len(pts_in_fire) != len(self.cells[cells].vertex_pts) and len(pts_in_fire) != 0:
                self.cells_in_abstract_total.append(cells)
            if pts_in_fire:
                cells_abstracted.append(cells)

        self.cells_with_abstractions = cells_abstracted
        self.fire_abstract_total = new_total

        for n, fires in enumerate(self.fire_sim.fire_list):
            for idx, pts in enumerate(fires[len(fires)-1]):
                cell_loc = (int(round(divmod(pts[0]+5.0, 10.0)[0])),int(round(divmod(pts[1]+5.0, 10.0)[0])))
                if cell_loc not in self.cells_with_abstractions and cell_loc not in self.obs_abstraction:
                    self.cells_with_abstractions.append(cell_loc)
                if cell_loc not in self.cells_in_abstract_total and cell_loc not in self.obs_abstraction:
                    self.cells_in_abstract_total.append(cell_loc)


    def update_suppressant_obstacles(self, time):
        temp_dict = {}
        check = False
        for obs in self.suppressant_obstacles:
            if time < self.suppressant_obstacles[obs][1][0] + self.suppressant_obstacles[obs][1][1]:
                temp_dict.update({obs: self.suppressant_obstacles[obs]})
            else:
                self.obstacles.remove(self.suppressant_obstacles[obs][0])
                check = True
        if check is True:
            self.suppressant_obstacles = temp_dict

    def update_abstractions_visuals(self, window_size, screen, pygame_instance, resolution):
        def transform_pt_to_pixels(ratio, domain, pts, window_size):
            new_pts = [int(round((pts[0]-domain[0][0])*ratio[0])),
                           window_size[1] - int(round((pts[1]-domain[1][0])*ratio[1]))]
            return new_pts

        m_to_pixel = (window_size[0]/(self.domain[0][1] - self.domain[0][0]),
                      window_size[1]/(self.domain[1][1] - self.domain[1][0]))

        for pts in self.fire_abstract_total:
            pygame_instance.draw.circle(screen, (160, 40, 40),
                                transform_pt_to_pixels(m_to_pixel, self.domain, pts,
                                         window_size), 2)

    def update_suppressant_visuals(self, window_size, screen, pygame_instance):
        def transform_vertex_to_pixels(ratio, domain, vertex, window_size):
            new_poly = list()
            for pts in vertex:
                #print(pts)
                #print('HELLO')
                new_pts = [int(round((pts[0]-domain[0][0])*ratio[0])),
                           window_size[1] - int(round((pts[1]-domain[1][0])*ratio[1]))]
                new_poly.append(new_pts)
            return new_poly

        m_to_pixel = (window_size[0]/(self.domain[0][1] - self.domain[0][0]),
                      window_size[1]/(self.domain[1][1] - self.domain[1][0]))

        for obs in self.suppressant_obstacles:
            pygame_instance.draw.polygon(screen, (165, 255, 176), transform_vertex_to_pixels(m_to_pixel,
                                        self.domain, self.suppressant_obstacles[obs][0],
                                         window_size))

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
