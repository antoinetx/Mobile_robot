import math
import numpy as np
from matplotlib import colors

class Map :
    def __init__(self, lenght_in_m, nb_square_per_side):
        self.lenght_m = lenght_in_m
        self.nb_square_per_side = nb_square_per_side
        self.grid = np.zeros((nb_square_per_side,nb_square_per_side))
        self.square_lenght = self.lenght_m/self.nb_square_per_side
        self.path = np.empty
        self.visit_node = np.empty
        self.cmap = colors.ListedColormap(['white', 'red'])

    def get_map(self):
        return self.grid

    def set_map(self, grid):
        self.grid = grid

    def update_map(self, new_grid):
        self.grid = new_grid

    def get_lenght(self):
        return self.nb_square_per_side

    def get_path(self):
        return self.path

    def set_path(self, path):
        self.path = path

    def get_visit_nodes(self):
        return self.path

    def set_visit_nodes(self, visit_node):
        self.visit_node = visit_node

    def get_cmap(self):
        return self.cmap