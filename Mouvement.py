

from asgiref.sync import sync_to_async
import tdmclient
import matplotlib.pyplot as plt
import numpy as np
from random import random


test_functions = True
# ## Move from point A to point B

# simulation parameters 
KP_dist = 9
KP_alpha = 30
dt = 0.01
BASICSPEED = 100
GAIN = 10

WIDTH_ROBOT = 10
MAX_Y = 480
MAX_X = 640



@tdmclient.notebook.sync_var
def initialization_motor(x,y,theta):
     # initialisation_motors
    if (theta <= np.pi/2 and theta >= 0):
        y_left = y + abs(np.cos(theta))*(WIDTH_ROBOT/2)
        x_left = x - abs(np.sin(theta))*(WIDTH_ROBOT/2)
        y_right = y - abs(np.cos(theta))*(WIDTH_ROBOT/2)
        x_right = x + abs(np.sin(theta))*(WIDTH_ROBOT/2)
    
    if (theta <= np.pi and theta >= np.pi/2):
        y_left = y - abs(np.cos(np.pi-theta))*(WIDTH_ROBOT/2)
        x_left = x - abs(np.sin(np.pi-theta))*(WIDTH_ROBOT/2)
        y_right = y + abs(np.cos(np.pi-theta))*(WIDTH_ROBOT/2)
        x_right = x + abs(np.sin(np.pi-theta))*(WIDTH_ROBOT/2)
        
    if (theta >= -np.pi/2 and theta <= 0):
        y_left = y + abs(np.cos(theta))*(WIDTH_ROBOT/2)
        x_left = x + abs(np.sin(theta))*(WIDTH_ROBOT/2)
        y_right = y - abs(np.cos(theta))*(WIDTH_ROBOT/2)
        x_right = x - abs(np.sin(theta))*(WIDTH_ROBOT/2)
        
    if (theta <= -np.pi/2 and theta >= -np.pi):
        y_left = y - abs(np.sin(theta- np.pi/2))*(WIDTH_ROBOT/2)
        x_left = x + abs(np.cos(theta- np,pi/2))*(WIDTH_ROBOT/2)
        y_right = y + abs(np.sin(theta-np.pi/2))*(WIDTH_ROBOT/2)
        x_right = x - abs(np.sin(theta - np.pi/2))*(WIDTH_ROBOT/2)

        
    return x_right, x_left, y_right, y_left

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
    beta is the angle between the robot's position and the goal position + goal angle
    
    Kp_dist * dist and Kp_alpha * alpha drive the robot along a line towards the goal

    """
    # initialisation_center
    x = x_robot
    y = y_robot
    theta = angle_robot # voir comment imposer entre [-pi, pi]
    
    #x_right, x_left , y_right, y_left = initialization_motor(x,y,theta)
        
    # distance computation respectively to the center
    dist_center = compute_distance(x_goal, y_goal, x, y)
    x_traj, y_traj = [], []
    
    # distance computation respectively to the right wheel    
    #dist_right = compute_distance(x_goal, y_goal, x_right, y_right) 
    x_traj_right, y_traj_right = [], []
    
    # distance computation respectively to the left wheel 
    #dist_left = compute_distance(x_goal, y_goal, x_left, y_left)
    x_traj_left, y_traj_left = [], []
    
    while dist_center > 1 :
        x_traj.append(x)
        y_traj.append(y)

        # update the distance 
        dist_center = compute_distance(x_goal, y_goal, x, y)
        #dist_right = compute_distance(x_goal, y_goal, x_right, y_right)
        #dist_left = compute_distance(x_goal, y_goal, x_left, y_left)

        # definition of angle alpha
        x_diff = x_goal - x
        y_diff = y_goal - y
        alpha = ( np.arctan2(y_diff, x_diff) - theta +np.pi) % (2*np.pi) - np.pi

        v = KP_dist * dist_center
        w = KP_alpha * alpha
        
        #speed_l = int(BASICSPEED + GAIN * KP_dist * dist_left)
        #speed_r = int(BASICSPEED + GAIN * KP_dist * dist_right)
        
        speed_l = int(BASICSPEED + GAIN * KP_dist * dist_center)
        speed_r = int(BASICSPEED - GAIN * KP_dist * dist_center)

        theta = theta + w * dt
        x = x + v * np.cos(theta) * dt
        y = y + v * np.sin(theta) * dt
        #x_right, x_left , y_right, y_left = initialization_motor(x,y,theta)
                    
        

    if dist_center <= 1 :
        speed_l = 0
        speed_r = 0

    return speed_l, speed_r 







