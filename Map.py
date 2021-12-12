# Map class
# 
# Author: Antoine Perrin, Robotic MA1, Fall 2021


import numpy as np
import math
import matplotlib.pyplot as plt
import cv2


class Map :
    def __init__(self, lenght_in_m, wanted_nb_square_per_side):
        """
        Init the Map object
        :param lenght_in_m: lenth of the y side in m 
        :wanted_nb_square_per_side: the number of square for the y axis
        """
        self._lenght_m = lenght_in_m #lenght of the smallest size
        self._wanted_nb_square_by_side = wanted_nb_square_per_side
        self._grid_init = False
        self._square_size_m = lenght_in_m / wanted_nb_square_per_side
        self._pourcentage = 1
        self.map_lenght_in_square = (0,0)
        self.pixel_to_m = 0
    

    def get_map(self):
        
        if self._grid_init:
            return self._grid
        else:
            print("No grid yet. Please init the grid")
            

    def set_map_lenght(self, frame):
        """
        Init some Map's parameters(_pourcentage, width, height, map_lenght_in_square, pixel_to_m)
        :param frame: the video frame
        """
        pourcentage = (self._wanted_nb_square_by_side/frame.shape[0])
        width = int(frame.shape[1] * pourcentage )
        height = int(frame.shape[0] * pourcentage )  
        self.map_lenght_in_square = (width,height)
        self._pourcentage = pourcentage
        self.pixel_to_m = self._lenght_m/frame.shape[0]

        
    def get_pourcentage(self):
        return self._pourcentage

    def get_lenght(self):
        return self.map_lenght_in_square
        
    
    def security_grid_expand(self, frame, mask_green, robot_len = 0.05, security_margin = 0.07):
        """
        Expand the grid to avoid the robot colyding whit an obstacle
        :param frame: The bleu(obstacle) mask 
        :param mask green: The green(parking slot) mask
        :param robot lenght: The width of the robot in m
        :param security margin: The margin we add to securize the robot
        :return: The new expand grid
        """
        
        high_value = 255
        check_treshold = 50
        
        robot_lenght = robot_len/self.pixel_to_m
        marge = security_margin/self.pixel_to_m 
        sec_square = math.ceil((robot_lenght/2)) + math.ceil(marge)

        len_i = len(frame)
        len_j = len(frame[0])
        
        new_frame = np.zeros((len_i,len_j))
        
        # For each pixel that have a value equal more than 50 we put all the value arround to 255.
        for i in range(len_i):
            if sum(frame[i,:] != 0): # this if allow to avoid the second loop if there is no obstacle on this line
                for j in range(len_j):
                    if (frame[i][j] > check_treshold):
                        new_frame[(i-sec_square):(i+1+sec_square),(j-sec_square):(j+1+sec_square)] = high_value

        # We add the map mask on the new frame to let the parking slot free
        idx = np.where(mask_green == high_value)
        new_frame[idx] = 0
        
        return new_frame
                            
                        
    
    def init_grid(self, frame, mask_green):
        """
        Initialise the grid from the given frame
        :param frame: The bleu(obstacle) mask 
        :param mask green: The green(parking slot) mask
        :save: Save the grid in the Map object.
        """
        # Put the masks in the right way
        frame =np.flipud(frame)
        frame = np.transpose(frame)
        mask_green =np.flipud(mask_green)
        mask_green = np.transpose(mask_green)
        pourcentage = self._pourcentage
        dim_t = self.map_lenght_in_square

        
        #Compute the secured frame
        secured_frame = self.security_grid_expand(frame, mask_green)
        
        # Decomment to save the secured frmae in jpg format
        cv2.imwrite("secured.jpg", secured_frame)
        
        # resize image
        dim = (dim_t[1],dim_t[0])
        resized_frame = cv2.resize(secured_frame, dim, interpolation = cv2.INTER_AREA)
        
        # Decomment to save the secured frmae in jpg format
        cv2.imwrite("rezize.jpg", resized_frame)
        
        self._grid = resized_frame
        
        # Set the grid init state as True
        self._grid_init = True
        
    
    def grid_show(self):
        """
        Plot the _grid
        """
        plt.imshow(self._grid, vmin=0, vmax=1, origin='lower', interpolation='none', alpha=1)
        plt.draw()
        plt.show()


    
    