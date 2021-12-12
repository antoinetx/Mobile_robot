import cv2
from Map import Map
#from Detector import detect_inrange, detect_center, detect_obstacle
from KalmanFilter import KalmanFilter
import sys
import math
import numpy as np
import numpy.linalg as LA
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.colors import ListedColormap


np.set_printoptions(threshold=sys.maxsize)

blue = 110
green = 60
red = 160 # ou 10
ROUGE = (0, 0, 255)
GREEN = (0, 255, 0)
BLEU = (255, 0, 0)

sensitivity = 20

#blue_lo=np.array([100, 50, 50])
#blue_hi=np.array([140, 255, 255])
#green_lo=np.array([40, 50, 50])
#green_hi=np.array([80, 255, 255])

class Pose:
    x = -1
    y = -1
    angle = 0
    
class Position:
    x= -1
    y=- 1



# ---- variable grobale
#KF=KalmanFilter(0.1, [0, 0])



pose_robot_1 = Pose
pose_robot_2 = Pose
vector = Position
goal = Position


# ------ display function ------
def put_center_circle(image, contours,points,color):
    #put a cicrcle on the center of the object
    center_points, center_contours = detect_center(image, contours)
    if (len(points)>0):
        for i in points:
            cv2.circle(image, (i[0], i[1]), 7, color, -1)
            
# ------ detector function -------

def mask_function(image, lo, hi):
    image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image=cv2.blur(image, (5, 5))
    mask=cv2.inRange(image, lo, hi)
    mask=cv2.erode(mask, None, iterations=8)
    mask=cv2.dilate(mask, None, iterations=6)
    return mask

def detect_inrange(image, surface, color):
    points=[]
    lo=np.array([color - sensitivity, 60, 60])
    hi=np.array([color + sensitivity, 255, 255])

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

def detect_center(image, contours):
    a=0
    center_points=[]
    center_contours=[]
    
    for i in contours:
        M = cv2.moments(i)
        if M['m00'] != 0:
            #print('valeur')
            #print(a)
            a += 1
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            center_points.append(np.array([int(cx), int(cy)]))
            center_contours.append(i)
 
    return center_points, center_contours

# ------ maths function --------

def angle_of_vectors(a,b,c,d):
       
    vec_a = np.array([a, b])
    vec_b = np.array([c, d])

    inner = np.inner(vec_a, vec_b)
    norms = LA.norm(vec_a) * LA.norm(vec_b)

    cos = inner / norms
    rad = np.arccos(np.clip(cos, -1.0, 1.0))
    
    if b > 0:
        rad = rad * (-1)
    
    return rad

# ------ vision function  -------
            
def init_goal(frame, factor_reduc):
    
    goal_vect = (0, 0)
    gr_points,  gr_mask, gr_contours=detect_inrange(frame, 1000, green)
    #print(' la position goal')
    #print(gr_points)
    
    
    if (len(gr_points)>0):
        goal.x = gr_points[0][0]
        goal.y = frame.shape[0] - gr_points[0][1]
    else:
        assert False, 'No parking slot free'
        
    goal_vect = (int(goal.x*factor_reduc), int(goal.y*factor_reduc))
    
    
    return goal_vect


    
def vision_end(VideoCap):
    VideoCap.release()
    cv2.destroyAllWindows()
    
def mask_map_init(frame):
    
    bl_points, bl_mask, bl_contours=detect_inrange(frame, 10000, blue)
    gr_points, gr_mask, gr_contours=detect_inrange(frame, 10000, green)
    
    
    return bl_mask, gr_mask
    


def setup_robot_pose(red_contours, red_points, size_frame):
    #if cv2.contourArea(red_contours[0]) > cv2.contourArea(red_contours[1]):
    if cv2.contourArea(red_contours[1]) > cv2.contourArea(red_contours[0]):
        #print('if')
        #calcul position
        pose_robot_1.x = red_points[0][0]
        pose_robot_1.y = size_frame - red_points[0][1]
        #print('x y du robot')
        #print(pose_robot_1.x)
        #print(pose_robot_1.y)
        
        #calcul vecteur
        pose_robot_2.x = red_points[1][0]
        pose_robot_2.y = size_frame -  red_points[1][1]
        #print(red_points[1][0])
        #print(pose_robot_2.x)
    else:
        #print('else')
        #calcul position
        pose_robot_1.x = red_points[1][0]
        pose_robot_1.y = size_frame - red_points[1][1]
        #print(red_points[0][0])
        
        #calcul vecteur
        pose_robot_2.x = red_points[0][0]
        pose_robot_2.y = size_frame - red_points[0][1]
        #print('x y du robot')
        #print(pose_robot_1.x)
        #print(pose_robot_1.y)
        
    
    #vecteur 
    vector.x = red_points[1][0] - red_points[0][0]
    vector.y = red_points[1][1] - red_points[0][1]
    angle = angle_of_vectors(vector.x,vector.y,1,0)
    pose_robot_1.angle = angle
    #print('l angle')
    #print(angle)
    
def update(frame, factor_reduc):
            
    size_frame = frame.shape[0]
    red_points, red_mask, red_contours = detect_inrange(frame, 200, red)
    
    #print("Hello2")
    #cv2.imshow("mask rouge",red_mask)
    
    if(len(red_points)>1):
        cnt = sorted(red_contours, key=cv2.contourArea, reverse=True)
    
        ((x, y), rayon)=cv2.minEnclosingCircle(cnt[0])
        red_points[0][0] = x
        red_points[0][1] = y

        ((x, y), rayon)=cv2.minEnclosingCircle(cnt[1])
        red_points[1][0] = x
        red_points[1][1] = y
        
        cv2.circle(frame, (red_points[0][0], red_points[0][1]), 10, BLEU, 5)
        cv2.circle(frame, (red_points[1][0], red_points[1][1]), 10, ROUGE, 5)
        #put_center_circle(frame,red_contours[1], red_points[1], ROUGE)
        setup_robot_pose(red_contours, red_points, size_frame)
    
    #display the vector and points
    
        
    #get the points of the robot
    if(len(red_points)>1):
        cv2.arrowedLine(frame,
                    (int(red_points[0][0]), int(red_points[0][1])), (int(red_points[1][0]), int(red_points[1][1])),
                    color=(0, 255, 0),
                    thickness=3,
                    tipLength=0.2)
        #print('arrowed')
      
      
    #etat=KF.predict().astype(np.int32)
    
    # calculate the position
    #if(len(red_points)>0):
        #cv2.circle(frame, (red_points[0][0], red_points[0][1]), 10, (0,255,0), 5)
        #KF.update(np.expand_dims(red_points[0],axis=-1))
        
    #if(len(red_points)>1):
        #print('robot detected')
        #setup_robot_pose(red_contours, red_points, size_frame)
            
    
    return  (int(pose_robot_1.x * factor_reduc), int(pose_robot_1.y*factor_reduc)), pose_robot_1.angle
    
    
    
    
def display (frame, bool_bl, bool_gr, bool_red, bool_path, path, factor_reduc):
    
    if bool_bl:
        bl_points, bl_mask, bl_contours = detect_inrange(frame, 1000, blue)
        put_center_circle(frame,bl_contours, bl_points, GREEN)
        
    if bool_gr:
        gr_points, gr_mask, gr_contours = detect_inrange(frame, 1000, green)
        put_center_circle(frame,gr_contours, gr_points, GREEN)
        
    #if bool_red:
        #red_points, red_mask, red_contours = detect_inrange(frame, 50, red)
        #put_center_circle(frame,red_contours[0], red_points[0], ROUGE)
       # put_center_circle(frame,red_contours[1], red_points[1], BLEU)
       # if(len(red_points)>1):
           # cv2.arrowedLine(frame,
           #         (int(red_points[0][0]), int(red_points[0][1])), (int(red_points[1][0]), int(red_points[1][1])),
           #         color=(0, 255, 0),
           #         thickness=3,
           #         tipLength=0.2)
        #print('arrowed')
        
    
    if bool_path:
        path_ar = np.transpose(path)
        if (len(path_ar)>0):
            for point in path_ar:
                cv2.circle(frame, (int(point[0]/factor_reduc), frame.shape[0] - int(point[1]/factor_reduc)), 7, BLEU, -1)
              
    
    cv2.imshow('image', frame)
    
    
    
        




     

        