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
        
        pourcentage = (self._wanted_nb_square_by_side/frame.shape[0])
        width = int(frame.shape[1] * pourcentage )
        height = int(frame.shape[0] * pourcentage )  
        self.map_lenght_in_square = (width,height)
        self._pourcentage = pourcentage
        self.pixel_to_m = self._lenght_m/frame.shape[0]

        
    def get_pourcentage(self):
        return self._pourcentage

    #def update_map(self, new_grid):
    #    self._grid = new_grid

    def get_lenght(self):
        return self.map_lenght_in_square
        
    
    
    def security_grid_expand(self, frame, robot_len = 0.05, security_margin = 0.03):
        """
        Expand the grid to avoid the robot colyding whit an obstacle
        :param frame: the video frame 
        :save in map._grid: save the new expand grid
        :return: the new expand grid
        """
        robot_lenght = robot_len/self.pixel_to_m # 10 cm IN PIXEL
        
        marge = security_margin/self.pixel_to_m # 3 cm de marge
        sec_square = math.ceil((robot_lenght/2)) + math.ceil(marge)

        len_i = len(frame)
        len_j = len(frame[0])
        
        new_frame = np.zeros((len_i,len_j))
        
        for i in range(len_i):
            if sum(frame[i,:] != 0): # this if allow to avoid the second loop if there is no obstacle on this line
                for j in range(len_j):
                    if frame[i][j] > 50:
                        new_frame[(i-sec_square):(i+1+sec_square),(j-sec_square):(j+1+sec_square)] = 255
        return new_frame
                            
                        
    
    def init_grid(self, frame):
        
        frame =np.flipud(frame)
        frame = np.transpose(frame)
        pourcentage = self._pourcentage
        dim_t = self.map_lenght_in_square


        secured_frame = self.security_grid_expand(frame)
        #cv2.imshow("secured", secured_frame)
        cv2.imwrite("secured.jpg", secured_frame)
        
        # resize image
        dim = (dim_t[1],dim_t[0])
        resized_frame = cv2.resize(secured_frame, dim, interpolation = cv2.INTER_AREA)
        
        #print('Resized Dimensions : ',resized_frame.shape)
        
        #cv2.imshow("rezize mask", resized_frame)
        cv2.imwrite("rezize.jpg", resized_frame)
        
        """
        secured_frame = self.security_grid_expand(resized_frame)
        #cv2.imshow("secured", secured_frame)
        cv2.imwrite("secured.jpg", secured_frame)
        """
        self._grid = resized_frame
        
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