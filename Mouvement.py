

from asgiref.sync import sync_to_async
import tdmclient
import matplotlib.pyplot as plt
import numpy as np
from random import random


test_functions = True
# ## Move from point A to point B

# simulation parameters 
KP_dist = 2
KP_alpha = 40
dt = 0.01
BASICSPEED = 100
GAIN = 10
MAX_SPEED = 200
th_dist = 10

@tdmclient.notebook.sync_var
def compute_distance(x_goal, y_goal, x, y):
    x_diff = x_goal - x
    y_diff = y_goal - y
    dist = np.hypot(x_diff, y_diff)
    
    return dist

@tdmclient.notebook.sync_var
def move_to_position(x_robot, y_robot, angle_robot, x_goal, y_goal):
    """
    dist is the distance between the robot and the goal position
    alpha is the angle to the goal respectively to the angle of the robot
    
    Kp_dist * dist and Kp_alpha * alpha drive the robot along a line towards the goal

    """

    print('debut',x_robot,y_robot,angle_robot)
        
    # distance computation respectively to the center
    dist_center = compute_distance(x_goal, y_goal, x_robot, y_robot)
    print('dist_debut',dist_center)


    # definition of angle alpha
    x_diff = x_goal - x_robot
    y_diff = y_goal - y_robot
    alpha = ( np.arctan2(y_diff, x_diff) - angle_robot +np.pi) % (2*np.pi) - np.pi

    # speed update
    v = KP_dist * dist_center
    print('v', v)
    w = KP_alpha * alpha
    print('w', w)

    if alpha > np.pi/18 :
        v = 0

    speed_r = int(v - w)
    speed_l = int(v + w)
    if speed_r > MAX_SPEED :
        speed_r = MAX_SPEED
    if speed_l > MAX_SPEED:
        speed_l = MAX_SPEED
    if dist_center <= th_dist:
        speed_l, speed_r = 0,0 

    print('speed,' , speed_l, speed_r)

    return speed_l, speed_r 



