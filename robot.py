import numpy as np


class Robot :

    test_h = 0
    test_coords = []

    def __init__(self):
            self.path = np.empty
            self.visit_node = np.empty
            self.current_path_index = 0
            self.start_pos = (-1,-1)
            self.pos = (-1,-1)
            self.goal = (0,0)
            self.angle = 0 # angle with y
            self.pos_err = 2 # Square error for position
    

    def get_pos(self):
        return self.pos
    def set_pos(self, pos):
        self.pos = pos
        
    def set_start_pos(self, pos):
        self.start_pos = pos

    def get_start(self):
        return self.start_pos

    def get_goal(self):
        return self.goal
    def set_goal(self, goal):
        self.goal = goal
        
    def get_angle(self):
        return self.angle 
        
    def set_angle(self, angle):
        self.angle = angle

    def get_path(self):
        return self.path
    def set_path(self, path):
        self.path = path

    def get_visit_nodes(self):
        return self.visit_node

    def set_visit_nodes(self, visit_node):
        self.visit_node = visit_node
        
    def get_err_pos(self):
        return self.pos_err
    
    def get_current(self):
        return self.current_path_index
    
    def set_current(self, value):
        self.current_path_index = value
    
    



