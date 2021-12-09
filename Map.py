import numpy as np
import math
import matplotlib.pyplot as plt
import cv2


class Map :
    def __init__(self, lenght_in_m, wanted_nb_square_per_side):
        """
        Init the Map object
        :param lenght_in_m: lenth of the smallest side in m 
        :wanted_nb_square_per_side: the approximately wanted nb of square for smallest side)
        """
        self._lenght_m = lenght_in_m #lenght of the smallest size
        self._wanted_nb_square_by_side = wanted_nb_square_per_side
        self._grid_init = False
        self._square_size_m = 0.1
        self._pourcentage = 1
    

    def get_map(self):
        if self._grid_init:
            return self._grid
        else:
            print("No grid yet. Please init the grid")
            

    def set_pourcentage(self, frame):
        self._pourcentage = (self._wanted_nb_square_by_side/frame.shape[0])
        
    def get_pourcentage(self):
        return self._pourcentage

    #def update_map(self, new_grid):
    #    self._grid = new_grid

    #def get_lenght(self):
        #return self._nb_square_by_side
        
    
    
    def security_grid_expand(self, frame, robot_len = 0.10, security_margin = 0.03):
        """
        Expand the grid to avoid the robot colyding whit an obstacle
        :param frame: the video frame 
        :save in map._grid: save the new expand grid
        :return: the new expand grid
        """
        robot_lenght = robot_len # 10 cm
        marge = security_margin # 3 cm de marge
        sec_square = math.ceil((robot_lenght/2)/self._square_size_m) + math.ceil(marge/self._square_size_m)

        len_i = len(frame)
        len_j = len(frame[0])
        
        new_frame = np.zeros((len_i,len_j))

        for i in range(len_i):
            if sum(frame[i,:] != 0): # this if allow to avoid the second loop if there is no obstacle on this line
                for j in range(len_j):
                    if frame[i][j] > 10:
                        new_frame[(i-sec_square):(i+sec_square),(j-sec_square):(j+sec_square)] = 1

        self._grid = new_frame # Save the new grid in the object
        return new_frame
    
    
    def init_grid(self, frame):
        
        pourcentage = self._pourcentage

        width = int(frame.shape[1] * pourcentage )
        height = int(frame.shape[0] * pourcentage )
        dim = (width, height)

        # resize image
        resized_frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
        
        print('Resized Dimensions : ',resized_frame.shape)
        
        cv2.imshow("rezize mask", resized_frame)
        
        
    
        secured_frame = self.security_grid_expand(resized_frame)
        
        self._grid = secured_frame
        self._pourcentage = pourcentage
        
        self._grid_init = True
        
    
    def grid_show(self):
        """
        Plot the _grid
        """
        plt.imshow(self._grid, vmin=0, vmax=1, origin='lower', interpolation='none', alpha=1)
        plt.draw()
        plt.show()


    
    
    
    
    """
    def init_grid2(self, frame, r_tresh, g_tresh, b_tresh):
    """
    #Create an OccupancyGridMap from a video frame
    #:param frame: the video frame 
    #:param r_tresh, g_tresh, b_tresh: the treshold to choose wich color to extract (each tes between 0and 255)
    #:Save in map._grid: the created grid map
    """
        self._square_pixe_size = math.floor(len(frame)/self._wanted_nb_square_by_side)
        self._nb_square_by_side = math.ceil(len(frame)/self._square_pixe_size)
        self._square_size_m = self._lenght_m/self._nb_square_by_side # Not the exact value 
        
        square_pixe_size = self._square_pixe_size
        nb_square_by_side = self._nb_square_by_side
        
        data_j = np.zeros((len(frame),math.ceil(len(frame[0])/square_pixe_size))) 
        data_i = np.zeros((math.ceil(nb_square_by_side),math.ceil(len(frame[0])/square_pixe_size)))      
        
        j_pixel_state = 0
        i_pixel_state = 0
        
        for i in range(len(frame)):
            for j in range(len(frame[0])):
                # order : b,g,r
                b = frame[i,j,0]
                g = frame[i,j,1]
                r = frame[i,j,2]
                
                if(((r >= r_tresh[0]) & (r <= r_tresh[1])) & ((g >= g_tresh[0]) & (g <= g_tresh[1])) & ((b >= b_tresh[0]) & (b <= b_tresh[1]))):
                    j_pixel_state = 1
                if (j % square_pixe_size == square_pixe_size-1):
                    
                    data_j[i,int(((j+1)/square_pixe_size)-1)] = j_pixel_state
                    j_pixel_state = 0
                elif ((j == len(frame[0]) - 1)):
                    data_j[i,math.ceil((j+1)/square_pixe_size)-1] = j_pixel_state
        
            
        print(data_j)

        #print(len(data_j[1]))
        print("half done")
        for j in range(len(data_j[1])):
            for i in range(len(frame)):
                if(data_j[i,j] == 1):
                    i_pixel_state = 1
                if (i % square_pixe_size == square_pixe_size-1):
                    data_i[(nb_square_by_side-1)-int(((i+1)/square_pixe_size)-1),j] = i_pixel_state
                    i_pixel_state = 0     
                elif ((i == len(frame) - 1)):
                    data_i[(nb_square_by_side-1)-(math.ceil((i+1)/square_pixe_size))-1,j] = i_pixel_state   
                    
                    
        # Save the data in the map variable _grid
        self._grid = data_i
        self._grid_init = True
    """