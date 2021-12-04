
import cv2
from Map import Map
from Detector import detect_inrange, detect_center
from KalmanFilter import KalmanFilter
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.colors import ListedColormap


np.set_printoptions(threshold=sys.maxsize)

blue = 120
green = 60
ROUGE = (0, 0, 255)
map = np.zeros((64,64))
reduction = 10
x_max = int(480)
y_max = int(640)





print('Hello World')

VideoCap=cv2.VideoCapture(1, cv2.CAP_DSHOW)

KF=KalmanFilter(0.1, [0, 0])


cmapmine = colors.ListedColormap(['w', 'b'])

while(True):
        
    ret, frame=VideoCap.read()
    
    points, mask, contours=detect_inrange(frame, 10000, blue)
    gr_points, gr_mask, gr_contours=detect_inrange(frame, 800, green)
    
    
    #put a cicrcle on the center of the object
    center_points, center_contours = detect_center(frame, contours)
    gr_center_points, gr_center_contours = detect_center(frame, gr_contours)
    for i in points:
        cv2.circle(frame, (i[0], i[1]), 7, (0, 0, 255), -1)
        
    for i in gr_points:
        cv2.circle(frame, (i[0], i[1]), 7, (0, 255, 0), -1)

    


    cv2.imshow('image', frame)
    
    if (len(points)>0):
        cv2.circle(frame, (points[0][0], points[0][1]), 10, (0, 255, 0), 2)
        
    
    #if mask is not None:
        # cv2.imshow('blue mask', mask)
        #cv2.imshow('green mask', green_mask)

    

    if cv2.waitKey(1)&0xFF==ord('q'):
        print('le bouton quitter')
        occupancy_grid=np.zeros((round(x_max/reduction),round(y_max/reduction)))
        print('la taille de l occupancy grid est')
        print(occupancy_grid.shape)

 
        map = np.zeros((480, 640))

        if (len(points)>0):
            for i in range(len(points)):
                x = int(480-points[i][1])
                y = points[i][0]
                map[ x, y] = 1
                occupancy_grid[int(x/reduction),int(y/reduction)]=1
                for v in range(int(x/reduction),int(x/reduction +10)):
                    for w in range(int(y/reduction),int(y/reduction +10)):
                        if (v-5) <48 or  (w-5)<64:
                            if occupancy_grid[int(v-5),int(w-5)] != 2:
                                occupancy_grid[int(v-5),int(w-5)]=1
                
                print(points[i][0], points[i][1])
        
        
        
        
        if (len(gr_points)>0):
            for i in range(len(gr_points)):
                x = int(480-gr_points[i][1])
                y = gr_points[i][0]
                map[ x,y] = 2
                occupancy_grid[int(x/reduction),int(y/reduction)]=0
                for v in range(int(x/reduction),int(x/reduction +10)):
                    for w in range(int(y/reduction),int(y/reduction +10)):
                        if (v-5) <48 or  (w-5)<64:
                            occupancy_grid[int(v-2),int(w-5)]=0
        
        
        print(' les centres des obstacles sont')
        print(points)
        print(map[ points[0][0], points[0][1] ])
        print(points[0][0])

        """
        
        for x in range(int(round(x_max)-1)):
            for y in range(int(round(y_max)-1)):
                # For each pixel, obstacle if color is white (255)
                
                if map[x,y] == 1:
                    occupancy_grid[int(x/reduction),int(y/reduction)]=1
                    #print(int(x/reduction),int(y/reduction)) 
                    for v in range(int(x/reduction),int(x/reduction +10)):
                        for w in range(int(y/reduction),int(y/reduction +10)):
                            if (v-5) <48 or  (w-5)<64:
                                if occupancy_grid[int(v-5),int(w-5)] != 2:
                                    occupancy_grid[int(v-5),int(w-5)]=1
                                    
                       
        for x in range(int(round(x_max)-1)):
            for y in range(int(round(y_max)-1)):
                # For each pixel, obstacle if color is white (255)       
                if map[x,y] == 2:
                    occupancy_grid[int(x/reduction),int(y/reduction)]=0
                    #print(int(x/reduction),int(y/reduction)) 
                    for v in range(int(x/reduction),int(x/reduction +4)):
                        for w in range(int(y/reduction),int(y/reduction +10)):
                            if (v-2) <48 or  (w-5)<64:
                                occupancy_grid[int(v-2),int(w-5)]=0
                
        """
                    
                       
        
        row = np.ones((16,64))
        print('row shape')
        print(row.shape)
        occupancy_grid = np.vstack((occupancy_grid, row))
        print('taille final')
        print(occupancy_grid.shape)
        print('affichage')
        #print(occupancy_grid[int(points[0][0]/reduction),int(points[0][1]/reduction)])
        #print(int(points[0][0]/reduction),int(points[0][1]/reduction))
        
        fig,  ax = plt.subplots(1)
        max_val = 64
        major_ticks = np.arange(0, max_val+1, 5)
        minor_ticks = np.arange(0, max_val+1, 1)
        ax.set_xticks(major_ticks)
        ax.set_xticks(minor_ticks, minor=True)
        ax.set_yticks(major_ticks)
        ax.set_yticks(minor_ticks, minor=True)
        ax.grid(which='minor', alpha=0.2)
        ax.grid(which='major', alpha=0.5)
        ax.set_ylim([-1,max_val])
        ax.set_xlim([-1,max_val])
        ax.grid(True)
        ax.imshow(occupancy_grid, cmap=cmapmine)#, vmin=0, vmax=1)
        ax.set_title('map')

        map = occupancy_grid
        plt.show()
                        
        VideoCap.release()
        cv2.destroyAllWindows()
        
        break    
