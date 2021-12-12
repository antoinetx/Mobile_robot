
# Implementation of **PD Controller**
# 
# Author: Nour Tnani, Robotic MA1, Fall 2021

from asgiref.sync import sync_to_async
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as LA


# simulation parameters 
KP_DIST     =   2
KP_ALPHA    =   50   
KD_DIST     =   0                  # We finally just needed a P controller for the distance
KD_ALPHA    =   15  
BASICSPEED  =   90
MAX_SPEED   =   200
DT          =   0.1


def compute_distance(x_goal, y_goal, x, y):
    """
    This function computes the distance between the distance
    :param x_goal       :   x coordinate of the goal 
    :param y_goal       :   y coordinate of the goal
    :param x            :   x coordinate of the robot
    :param y            :   y coordinate of the robot
    :return             :   distance between the goal and the robot 
    """
    x_diff = x_goal - x
    y_diff = y_goal - y
    dist = np.hypot(x_diff, y_diff)
    
    return dist

def unit_vector(vector):
    """
    This function normalizes a vector 
    :param vector       :   the vector expressed as an array
    :return             :   the vector with normalized coordinates
    """
    return vector / np.linalg.norm(vector)

def get_angle (axe_ref, vect_goal):
    """
    This function gives the angle between two desired vectors
    :param axe_ref      :   first vector expressed as an array, vector of reference 
    :param vect_goal    :   second vector expressed as an array, moving vector
    :return             :   angle between the vectors, comprised in the range [-pi, pi]         
    """
    v1_u = unit_vector(axe_ref)
    v2_u = unit_vector(vect_goal)
    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))           # gives an unsigned angle

    if vect_goal[1] < 0:
        angle = 2*np.pi - angle                                         # we do these changes to have the negative angle if the y coordinate of the goal is negative.
    return angle

def angle_voulu(angle_goal, angle_robot):
    """
    This function gives the optimal difference of angles
    :param angle_goal   :   angle of the desired goal, as computed in the get_angle function
    :param angle_robot  :   the actual angle that the robot has
    :return             :   the desired angle change that has to be done
    """
    angle_voulu = angle_goal - angle_robot                         
    if angle_voulu > np.pi:                                              # if the desired changing angle is more than 180°, the robot will rotate in the other way
        angle_voulu = angle_goal -2*np.pi - angle_robot
    return angle_voulu
    
def PD (old_value, new_value, Kd, Kp, dt) :
    """
    This function computes the Proportion Derivative controller of a given variable
    :param old_value    :   value of the variable at time t-1
    :param new_value    :   value of the variable at time t
    :param Kd           :   Kd parameter to tune with experiments
    :param Kp           :   Kp parameter to tune with experiments
    :param dt           :   time difference between each loop
    :return             :   the PD coefficient to consider
    """
    erreur = new_value - old_value
    D = (Kd * erreur) / dt
    P = Kp * new_value
    PD =  P + D
    return PD


def move_to_position(pos_robot , angle_robot, pos_goal, old_distance, old_angle):
    """
    This function will drive the robot along a line towards the goal
    :param pos_robot        :   array of the robot's coordinates
    :param angle_robot      :   value of the angle of the robot, given between [-pi,pi]
    :param pos_goal         :   array of the goal's coordinates
    :param old_distance     :   value of the last computed distance error, needed for PD implementation
    :param old_angle        :   value of the last computed angle error, needed for PD implementation
    :return speed_l         :   speed of the left wheel
    :return speed_r         :   speed of the right wheel
    :return old_distance    :   the error in distance that has just been computed
    :return old_angle       :   the error in angle that has just been computed 
    """
    x_robot, y_robot = pos_robot[0] , pos_robot[1]
    x_goal , y_goal = pos_goal[0] , pos_goal[1]
        
    # distance computation respectively to the center
    dist_center = compute_distance(x_goal, y_goal, x_robot, y_robot)  

    # computation of angle alpha
    axe_ref = np.array([1,0])
    vect_goal = np.array( [(x_goal - x_robot), (y_goal - y_robot)])
    angle_goal = get_angle(axe_ref, vect_goal)
    alpha = angle_voulu(angle_goal, angle_robot)   

    # Implementation of PD :
    v = PD (old_distance, dist_center, KD_DIST, KP_DIST, DT)
    w = PD (old_angle, alpha, KD_ALPHA, KP_ALPHA, DT)

    speed_r = int(BASICSPEED + v + w)
    speed_l = int(BASICSPEED + v - w)

    if alpha > np.pi/2 or alpha < -np.pi/2:
        speed_r = int(w)
        speed_l = int(- w)

    if speed_r > MAX_SPEED :
        speed_r = MAX_SPEED
    if speed_l > MAX_SPEED:
        speed_l = MAX_SPEED

    old_distance = dist_center
    old_angle = alpha
 
    return speed_l, speed_r , old_distance, old_angle


