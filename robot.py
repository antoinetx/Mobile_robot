import numpy as np


class Robot :

    test_h = 0
    test_coords = []

    def __init__(self, robot_start, goal):
            self.path = np.empty
            self.visit_node = np.empty
            self.current_path_index = 0
            self.start_pos = robot_start
            self.pos = robot_start
            self.goal = goal
            self.angle = 0 # angle with y
            self.pos_err = 2
    

    def get_pos(self):
        return self.pos
    def set_pos(self, pos):
        self.pos = pos

    def get_start(self):
        return self.start_pos

    def get_goal(self):
        return self.goal
    def set_goal(self, goal):
        self.goal = goal

    def get_path(self):
        return self.path
    def set_path(self, path):
        self.path = path

    def get_visit_nodes(self):
        return self.visit_node
    def set_visit_nodes(self, visit_node):
        self.visit_node = visit_node



