# Implementation of the  *Computer Vision*
# 
# Author:Xavier Nal, Robotic MA1, Fall 2021
#
# imports
import cv2
from Map import Map
from KalmanFilter import KalmanFilter
import sys
import math
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.colors import ListedColormap

#constants 
BLUE_HSV = 110
GREEN_HSV = 60
RED_HSV = 160 
SATURATION_HI = 255
SATURATION_LO = 60
VALUE_HI = 255
VALUE_LO = 60
SENSITIVITY = 20

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)

THRESHOLD_SURFACE_BLUE = 1000
THRESHOLD_SURFACE_GREEN = 1000
THRESHOLD_SURFACE_RED = 200


# class definition   
class Position:
    x= -1
    y=- 1

# variables grobales

robot_orientation = 0

point_robot_1 = np.array([-1,-1])
point_robot_2 = np.array([-1,-1])

goal = Position


# --------  Extern functions ------------

def update(frame, factor_reduc):
    """ function call in the main: in charge of 
    managing the steps of the vision. Take in input the 
    image and the factor of resize of the map """
    
    bool_mesure = False
    size_frame = frame.shape[0]
    red_points, red_mask, red_contours = detect_inrange(frame, THRESHOLD_SURFACE_RED, RED_HSV)
 
    #setup the robot position if we found two red points
    if(len(red_points)>1):
        bool_mesure = True
        cnt = sorted(red_contours, key=cv2.contourArea, reverse=True)
    
        ((x, y), rayon)=cv2.minEnclosingCircle(cnt[0])
        red_points[0][0] = x
        red_points[0][1] = y

        ((x, y), rayon)=cv2.minEnclosingCircle(cnt[1])
        red_points[1][0] = x
        red_points[1][1] = y
        
        cv2.circle(frame, (red_points[0][0], red_points[0][1]), 10, BLUE, 5)
        cv2.circle(frame, (red_points[1][0], red_points[1][1]), 10, RED, 5)

        setup_robot_pose(red_contours, red_points, size_frame)
        
    #show the vector of the position on the cv2 imshow
    if(len(red_points)>1):
        cv2.arrowedLine(frame,
                    (int(red_points[0][0]), int(red_points[0][1])), (int(red_points[1][0]), int(red_points[1][1])),
                    color= GREEN,
                    thickness=3,
                    tipLength=0.2)
  

    return  (int(point_robot_1[0] * factor_reduc), int(point_robot_1[1]*factor_reduc)), (int(point_robot_2[0] * factor_reduc), int(point_robot_2[1]*factor_reduc)), robot_orientation, bool_mesure

def mask_map_init(frame):
    """ do the init mask with the color blue and green
    for the initialisation of the map to find the parking
    slot and the obstacles """
    
    bl_points, bl_mask, bl_contours=detect_inrange(frame, 10000, BLUE_HSV)
    gr_points, gr_mask, gr_contours=detect_inrange(frame, 10000, GREEN_HSV)
    
    return bl_mask, gr_mask

def vision_end(VideoCap):
    """ close the video cap and the open windows"""
    VideoCap.release()
    cv2.destroyAllWindows()


def init_goal(frame, factor_reduc):
    """ find an parking slot free by exctracting green object 
    position with a green mask """
    
    goal_vect = (0, 0)
    gr_points,  gr_mask, gr_contours=detect_inrange(frame, THRESHOLD_SURFACE_GREEN, GREEN_HSV)
 
    if (len(gr_points)>0):
        goal.x = gr_points[0][0]
        goal.y = frame.shape[0] - gr_points[0][1]
    else:
        assert False, 'No parking slot free'
        
    goal_vect = (int(goal.x*factor_reduc), int(goal.y*factor_reduc))
    
    
    return goal_vect


def display (frame, bool_bl, bool_gr, bool_path, path, factor_reduc):
    """ display the frame with point on the object
        allow to show the path planning """
    
    if bool_bl:
        bl_points, bl_mask, bl_contours = detect_inrange(frame, THRESHOLD_SURFACE_BLUE, BLUE_HSV)
        put_center_circle(frame,bl_contours, bl_points, GREEN)
        
    if bool_gr:
        gr_points, gr_mask, gr_contours = detect_inrange(frame, THRESHOLD_SURFACE_GREEN, GREEN_HSV)
        put_center_circle(frame,gr_contours, gr_points, GREEN)
        
    if bool_path:
        path_ar = np.transpose(path)
        if (len(path_ar)>0):
            for point in path_ar:
                cv2.circle(frame, (int(point[0]/factor_reduc), frame.shape[0] - int(point[1]/factor_reduc)), 7, BLUE, -1)    
    
    cv2.imshow('image', frame)
    
    
# ------- internal functions -------

def setup_robot_pose(red_contours, red_points, size_frame):
    """ set up the global vector for the two red 
    points on the robot and find the orientation of the robot """
    
    #position calculation with the bigger red point in the first robot point
    if cv2.contourArea(red_contours[0]) > cv2.contourArea(red_contours[1]):
        point_robot_1[0] = red_points[0][0]
        point_robot_1[1] = size_frame - red_points[0][1]
        
        point_robot_2[0] = red_points[1][0]
        point_robot_2[1] = size_frame - red_points[1][1]

    else:
        point_robot_1[0] = red_points[1][0]
        point_robot_1[1] = size_frame - red_points[1][1]
        
        point_robot_2[0] = red_points[0][0]
        point_robot_2[1] = size_frame - red_points[0][1]    
      
    #vector position
    vector_position = Position
    vector_position.x = red_points[1][0] - red_points[0][0]
    vector_position.y = red_points[1][1] - red_points[0][1]
    
    #angle between x axis and the vector position
    angle = angle_of_vectors(vector_position.x,vector_position.y, 1,0)
    robot_orientation = angle
    

# ------ display function ------

def put_center_circle(image, contours,points,color):
    """ put a cicrcle on the center of the object """
    
    if (len(points)>0):
        for i in points:
            cv2.circle(image, (i[0], i[1]), 7, color, -1)
            

# ------ detector functions -------

def mask_function(image, lo, hi):
    """ opencv functions to extract a mask
      of one color from an image """
    
    image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image=cv2.blur(image, (5, 5))
    mask=cv2.inRange(image, lo, hi)
    mask=cv2.erode(mask, None, iterations=8)
    mask=cv2.dilate(mask, None, iterations=6)
    
    return mask

def detect_inrange(image, surface, color):
    """ detect the point, contours and mask from
     an object in an image with his color"""
    
    points=[]
    lo=np.array([color - SENSITIVITY, SATURATION_LO, VALUE_LO])
    hi=np.array([color + SENSITIVITY, SATURATION_HI, VALUE_HI])

    mask = mask_function(image, lo, hi)
    
    elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    elements=sorted(elements, key=lambda x:cv2.contourArea(x), reverse=True)
    
    for element in elements:
        if cv2.contourArea(element)>surface:
            ((x, y), rayon)=cv2.minEnclosingCircle(element)
            points.append(np.array([int(x), int(y)]))
        else:
            break

    return points, mask, elements

# ------ maths function --------

def angle_of_vectors(a,b,c,d):
    """ return the angle between two vecto """
       
    vec_a = np.array([a, b])
    vec_b = np.array([c, d])

    inner = np.inner(vec_a, vec_b)
    norms = LA.norm(vec_a) * LA.norm(vec_b)

    cos = inner / norms
    rad = np.arccos(np.clip(cos, -1.0, 1.0))
    
    # put the right sign for the angle
    if b > 0:
        rad = rad * (-1)
    
    return rad
