

from asgiref.sync import sync_to_async
import tdmclient
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as LA


# ## Move from point A to point B

# simulation parameters 
KP_dist = 2
KP_alpha = 70
BASICSPEED = 100
GAIN = 10
MAX_SPEED = 200
th_dist = 40

#@tdmclient.notebook.sync_var
def compute_distance(x_goal, y_goal, x, y):
    x_diff = x_goal - x
    y_diff = y_goal - y
    dist = np.hypot(x_diff, y_diff)
    
    return dist


#@tdmclient.notebook.sync_var
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
    angle_goal = np.arccos (1/dist_center)
    print('angle_goal,', angle_goal)
    alpha = abs(angle_goal - angle_robot)   # erreur d'angle à corriger avec le PD
    print('alpha', alpha)
    # speed update
    v = KP_dist * dist_center
    print('v', v)
    w = KP_alpha * alpha
    print('w', w)

    if alpha > np.pi/18 or alpha < -np.pi/18:
        v = 0
    print('v2', v)
    speed_r = int(v - w)
    speed_l = int(v + w)
    print('speed_original,' , speed_l, speed_r)
    if speed_r > MAX_SPEED :
        speed_r = MAX_SPEED
    if speed_l > MAX_SPEED:
        speed_l = MAX_SPEED
    if dist_center <= th_dist:
        speed_l, speed_r = 0,0 

    print('speed,' , speed_l, speed_r)

    return speed_l, speed_r 


move_to_position(4.7 , 23, 1.43, 40,40)

