import numpy as np



class Map :
    def __init__(self, lenght_in_m, nb_square_per_side):
            self.lenght_m = lenght_in_m
            self.nb_square_per_side = nb_square_per_side
            self.grid = np.empty((nb_square_per_side,nb_square_per_side))
            self.square_lenght = self.lenght_m/self.nb_square_per_side
            
            

    def get_map(self):
        return self.grid

    def update_map(self, new_grid):
        self.grid = new_grid

    def get_lenght(self):
        return self.nb_square_per_side