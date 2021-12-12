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

blue = 120
green = 60
red = 160 # ou 10
ROUGE = (0, 0, 255)
GREEN = (0, 255, 0)
BLEU = (255, 0, 0)
map = np.zeros((64,64))
reduction = 10
x_max = int(480)
y_max = int(640)

blue = 120
green = 60
sensitivity = 20

blue_lo=np.array([100, 60, 60])
blue_hi=np.array([140, 255, 255])

green_lo=np.array([40, 50, 50])
green_hi=np.array([80, 255, 255])

class Pose:
    x = 0
    y = 0
    angle = 0
    
class Position:
    x= 0
    y=0



# ---- variable grobale

KF=KalmanFilter(0.1, [0, 0])
stop_video = False


pose_robot_1 = Pose
pose_robot_2 = Pose
vector = Position
goal = Position



def put_center_circle(image, contours,points,color):
    #put a cicrcle on the center of the object
    center_points, center_contours = detect_center(image, contours)
    if (len(points)>0):
        for i in points:
            cv2.circle(image, (i[0], i[1]), 7, color, -1)
            
def vision_initialization(frame):
    
        
    gr_points,  gr_mask, gr_contours=detect_inrange(frame, 10000, green)
    if (len(gr_points)>0):
        goal.x = gr_points[0][0]
        goal.y =480 -gr_points[0][1]
    return gr_points, gr_mask
    
    
def vision_end(VideoCap):
    VideoCap.release()
    cv2.destroyAllWindows()
    
def mask_map_init(VideoCap):
    ret, frame=VideoCap.read()
    
    bl_points, bl_mask, bl_contours=detect_inrange(frame, 10000, blue)
    
    if (len(bl_points)>0):
        for i in bl_points:
            cv2.circle(frame, (i[0], i[1]), 7, (0, 0, 255), -1)
    return bl_mask
    
    
def mask_function(image, lo, hi):
    image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image=cv2.blur(image, (5, 5))
    mask=cv2.inRange(image, lo, hi)
    mask=cv2.erode(mask, None, iterations=4)
    mask=cv2.dilate(mask, None, iterations=4)
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
            """
            cv2.drawContours(image, [i], -1, (0, 255, 0), 2)
            cv2.circle(image, (cx, cy), 7, (0, 0, 255), -1)
            cv2.putText(image, "center", (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    """
    #print(f"x: {cx} y: {cy}")
    return center_points, center_contours

def get_pose():
    return pose_robot_1.x, pose_robot_1.y, pose_robot_1.angle

  
def get_goal(factor_reduc):
    return goal.x*factor_reduc, goal.y*factor_reduc

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

def setup_robot_pose(red_contours, red_points):
    if cv2.contourArea(red_contours[0]) > cv2.contourArea(red_contours[1]):
        #print('if')
        #calcul position
        pose_robot_1.x = red_points[0][0]
        pose_robot_1.y =480 - red_points[0][1]
        #print('x y du robot')
        #print(pose_robot_1.x)
        #print(pose_robot_1.y)
        
        #calcul vecteur
        pose_robot_2.x = red_points[1][0]
        pose_robot_2.y =480 -  red_points[1][1]
        #print(red_points[1][0])
        #print(pose_robot_2.x)
    else:
        #print('else')
        #calcul position
        pose_robot_1.x = red_points[0][0]
        pose_robot_1.y = 480 - red_points[0][1]
        #print(red_points[0][0])
        
        #calcul vecteur
        pose_robot_2.x = red_points[1][0]
        pose_robot_2.y = 480 - red_points[1][1]
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
    
def sorting_contour(points, elements):
    
    print('sort')
    
    if(len(points)>1):
        point_1 = []
        point_2 = []
        surface_1 = 0
        surface_2 = 0
        #if cv2.contourArea(red_contours[0]) > cv2.contourArea(red_contours[1]):
        i = 0
        for element in elements:
            if cv2.contourArea(element)>surface_2:
                surface_2 = cv2.contourArea(element)
                point_2 =  points[i]
                if surface_2 > surface_1:
                    surface_inter = surface_1
                    surface_1 = surface_2
                    surface_2 = surface_inter
                    
                    point_inter = point_1
                    point_1 = point_2
                    point_2 = point_inter
                
                
            
            i += 1
    else:
        point_1 = []
        point_2 = []
        surface_1 = 0
        surface_2 = 0
        
    
    return point_1, point_2, surface_1, surface_2
    
    
def update(frame, factor_reduc, kalman_bool):    
               
    red_points, red_mask, red_contours = detect_inrange(frame, 400, red) 
    
    if(len(red_points)>1):
        cnt = sorted(red_contours, key=cv2.contourArea, reverse=True)
    
        ((x, y), rayon)=cv2.minEnclosingCircle(cnt[0])
        red_points[0][0] = x
        red_points[0][1] = y

        ((x, y), rayon)=cv2.minEnclosingCircle(cnt[1])
        red_points[1][0] = x
        red_points[1][1] = y
        
        cv2.circle(frame, (red_points[0][0], red_points[0][1]), 10, BLEU, 5)
        put_center_circle(frame,red_contours, red_points, ROUGE)
        setup_robot_pose(red_contours, red_points)
        
    #get the points of the robot
    if(len(red_points)>1):
        cv2.arrowedLine(frame,
                    (int(red_points[0][0]), int(red_points[0][1])), (int(red_points[1][0]), int(red_points[1][1])),
                    color=(0, 255, 0),
                    thickness=3,
                    tipLength=0.2)

    cv2.imshow('image', frame)
    
    #get the points of the robot
    if(len(red_points)>1):
        cv2.arrowedLine(frame,
                    (int(red_points[0][0]), int(red_points[0][1])), (int(red_points[1][0]), int(red_points[1][1])),
                    color=(0, 255, 0),
                    thickness=3,
                    tipLength=0.2)
        #print('arrowed')
        
    etat=KF.predict().astype(np.int32)
      
    if(len(red_points)>0):
    
        if(len(red_points)>1):
            #print('robot detected')
            setup_robot_pose(red_contours, red_points)
            # show a vector for the orientation
                  
   # cv2.imshow('image', frame)
    
    if cv2.waitKey(1)&0xFF==ord('q'):
        print('le bouton quitter')
        
        vision_end(VideoCap)
        stop_video =True  
        
    
    return  pose_robot_1.x * factor_reduc, pose_robot_1.y*factor_reduc, pose_robot_1.angle

# ---- MAIN ----

VideoCap=cv2.VideoCapture(0)

KF=KalmanFilter(0.1, [0, 0])


while(True):
    ret, frame=VideoCap.read()
    
    #gr_points, gr_mask = vision_initialization(frame)
    
    pose_robot_1.x , pose_robot_1.y, pose_robot_1.angle= update(frame, 1, 0)
    
    bl_points, bl_mask, bl_contours = detect_inrange(frame, 1000, blue)
    gr_points, gr_mask, gr_contours = detect_inrange(frame, 1000, green)
    
    red_points, red_mask, red_contours = detect_inrange(frame, 400, red) 
    
    if(len(red_points)>1):
        cv2.arrowedLine(frame,
                    (int(red_points[0][0]), int(red_points[0][1])), (int(red_points[1][0]), int(red_points[1][1])),
                    color=(0, 255, 0),
                    thickness=3,
                    tipLength=0.2)
    
    cv2.imshow('image bl', bl_mask)
    cv2.imshow('image gr', gr_mask)
    cv2.imshow('image red', red_mask)
   # print('red')
    
    #put_center_circle(frame,bl_contours, bl_points, ROUGE)
    #put_center_circle(frame,gr_contours, gr_points, ROUGE)
    
    #image, cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #cnt = sorted(red_contours, key=cv2.contourArea)
    
          
    if cv2.waitKey(1)&0xFF==ord('q'):
        print('le bouton quitter')        
        vision_end(VideoCap)
        break 


    
# https://stackoverflow.com/questions/32669415/opencv-ordering-a-contours-by-area-python       