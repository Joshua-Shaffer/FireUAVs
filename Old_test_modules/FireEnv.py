'''
These two classes contain the environment as a whole and all individual cells in the environment. The update rules
regarding the cells still need to be added from Matlab code
'''
import math
from Fire_Model_Eqns import FireSimulator


class Env(object):
    def __init__(self, domain=[[0.0, 100.0], [0.0, 100.0]], starting_loc=[[30.1, 30.1]], burn_length=30.0, U=1.0, obstacle_list=[[]]):
        self.cells = {}
        self.fire_sim = FireSimulator(domain, starting_loc, burn_length, U=U)
        self.obstacles = obstacle_list

    def add_cell(self, key, *args):
        self.cells[key] = Cell(*args)

    def update_cells(self, params, time_step):

        # First loop used for updating the growth of fire in each cell
        for i in self.cells:
            # Fires only update in cells that aren't obstacles, aren't at the domain edge, and are greater than zero
            if (self.cells[i].fire > 0 and i[0] != 1 and i[0] != params.width and i[1] != 1 and
                    i[1] != params.height and self.cells[i].obstacle != 1 and self.cells[i].ext != 1):

                # Update fire growth counter
                self.cells[i].firetime = self.cells[i].firetime + time_step

                # Grow fire if the counter reaches growth time required
                if self.cells[i].fireupdate <= self.cells[i].firetime:
                    self.cells[i].set_fire_growth(params)
                else:
                    self.cells[i].update_cells = False

                # Extinguish fire if fuel dropped to zero
                if self.cells[i].fuel == 0:
                    self.cells[i].set_fire_extinguished(params)

        # Second loop is used to expand fires that have grown
        for i in self.cells:
            # Only apply from cells that have grown in fire
            if self.cells[i].update_cells is True:
                input('wait...')
                # Update cells adjacent to fire that grew
                update_keys = [(i[0] + 1, i[1]), (i[0] - 1, i[1]), (i[0], i[1] + 1), (i[0], i[1] - 1)]
                print(update_keys)
                for r in update_keys:
                    # Same conditions for cells that can hold fire as previous outer loop
                    print(self.cells[r].obstacle)
                    print(self.cells[r].fire)
                    print(self.cells[i].fire)
                    print(self.cells[r].ext)
                    input('wait...')
                    if (r[0] != 1 and r[0] != params.width and r[1] != 1 and
                            r[1] != params.height and self.cells[r].obstacle != 1 and
                            self.cells[r].fire < self.cells[i].fire and self.cells[r].ext == 0):
                        self.cells[r].set_fire_catching(params, self.cells[i].fire)
                        print(self.cells[r].fire)
                        print(self.cells[r].ext)

    def update_cells_agent_action(self, params):

        # Extinguish all cells that accumulate more water than necessary amount for the given fire level
        for i in self.cells:
            # In current step update, add the water accumulated to the total amount
            if self.cells[i].water_accum > 0.0:
                self.cells[i].water_accum_tot = self.cells[i].water_accum_tot + \
                                                self.cells[i].water_accum * (math.pow(
                    0.6 + self.cells[i].water_accum / params.max_water_capacity, 2))
                self.cells[i].water_accum = 0.0
            # If total surpasses the necessary amount, extinguish the fire
            if self.cells[i].water_accum_tot >= params.ext_vol[self.cells[i].fire] and \
                    self.cells[i].fire > 0:
                input('wait...')
                self.cells[i].set_fire_extinguished(params)


class Cell(object):
    def __init__(self, params, obs, fuel, fire, fireupdate, water_accum):
        self.water_accum = water_accum
        self.water_accum_tot = 0.0
        self.obstacle = obs
        self.fuel = fuel
        self.fire = fire
        self.firetime = 0.0
        self.fireupdate = params.fire_update_times[self.fire]
        self.ext = 0
        self.update_cells = False

    def __str__(self):
        return 'Water: ' + str(self.water_accum) + ' Water total: ' + str(self.water_accum_tot) + ' Obstacle: ' +\
               str(self.obstacle) + ' Fuel: ' + str(self.fuel) + ' Fire: ' + str(self.fire) + ' Fire time: ' +\
               str(self.firetime) + ' Time needed to update: ' + str(self.fireupdate) + ' Extinguished: ' +\
               str(self.ext)

    def __repr__(self):
        return self.__str__()

    def set_fire_extinguished(self, params):
        self.water_accum = 0.0
        self.water_accum_tot = 0.0
        self.fire = 0
        self.firetime = 0.0
        self.fireupdate = params.fire_update_times[self.fire]
        self.ext = 1
        self.update_cells = False

    def set_fire_growth(self, params):
        fuel_drop = self.fuel - self.fire
        self.fuel = 0 if fuel_drop < 0 else fuel_drop
        fire_rise = self.fire + 1
        self.fire = params.max_fire_intensity if fire_rise > params.max_fire_intensity else \
            fire_rise
        self.water_accum = 0.0
        self.water_accum_tot = 0.0
        self.firetime = 0.0
        self.fireupdate = params.fire_update_times[self.fire]
        self.ext = 0
        self.update_cells = True

    def set_fire_catching(self, params, fire_source):
        self.fire = params.max_fire_intensity if fire_source > params.max_fire_intensity else \
            fire_source
        self.water_accum = 0.0
        self.water_accum_tot = 0.0
        self.firetime = 0.0
        self.fireupdate = params.fire_update_times[self.fire]
        self.update_cells = False
