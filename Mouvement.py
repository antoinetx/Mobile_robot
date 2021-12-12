

from asgiref.sync import sync_to_async
import tdmclient
import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as LA


# ## Move from point A to point B

# simulation parameters 
KP_dist = 2
KP_alpha = 20   #90
BASICSPEED = 90
GAIN = 10
MAX_SPEED = 200
th_dist = 1
dt = 0.1
KD_dist = 0
KD_alpha = 50

#@tdmclient.notebook.sync_var
def compute_distance(x_goal, y_goal, x, y):
    x_diff = x_goal - x
    y_diff = y_goal - y
    dist = np.hypot(x_diff, y_diff)
    
    return dist

def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def get_angle (axe_ref, vect_goal):
    v1_u = unit_vector(axe_ref)
    v2_u = unit_vector(vect_goal)
    angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    if vect_goal[1] < 0:
        angle = 2*np.pi - angle
    return angle

def angle_voulu(angle_goal,angle_robot):
    angle_voulu = angle_goal - angle_robot
    print(angle_voulu)
    #cette condition fait que si l'angle entre le robot et le goal est plus grand que pi (180°)
    #il change l'angle goal pour que la rotation soit minimum
    # par exemple si on prends l'axe des x comme 0° si le robot a un angle de 90° et le goal un angle de 315° (270 + 45)
    # on a forcé dans la fonction au dessu l'angle a etre 315° et non -45° mais ici la rotation que le robot effectuerait 
    #serait donc de 225°, il vaudrait donc mieu utiliser -45° pour que le robot tourne dans l'autre sens et on prends donc 2pi - angle_goal
    #ce qui donne bien les -45°. De cette manière l'angle obtenu au final est tjr plus petit que 180°
    if angle_voulu > np.pi:
        angle_voulu = angle_goal -2*np.pi - angle_robot
    return angle_voulu
    
def PD (old_value, new_value,Kd,Kp, dt) :

    erreur = new_value - old_value
    D = (Kd * erreur) / dt
    P = Kp * new_value
    PD =  P + D
    return PD


#@tdmclient.notebook.sync_var
def move_to_position(pos_robot , angle_robot, pos_goal, old_distance, old_angle):
    """
    dist is the distance between the robot and the goal position
    alpha is the angle to the goal respectively to the angle of the robot
    
    Kp_dist * dist and Kp_alpha * alpha drive the robot along a line towards the goal

    """
    print ("movement")
    print (pos_robot)
    print('old dist,' , old_distance, 'old angle', old_angle)

    x_robot, y_robot = pos_robot[0] , pos_robot[1]
    x_goal , y_goal = pos_goal[0] , pos_goal[1]


    print('debut',x_robot,y_robot,angle_robot)
        
    # distance computation respectively to the center
    dist_center = compute_distance(x_goal, y_goal, x_robot, y_robot)
    print('dist_debut',dist_center)

    # computation alpha
    axe_ref = np.array([1,0])
    vect_goal = np.array( [(x_goal - x_robot), (y_goal - y_robot)])
    angle_goal = get_angle(axe_ref, vect_goal)
    print('angle_goal,', angle_goal)
    alpha = angle_voulu(angle_goal, angle_robot)   # erreur d'angle à corriger avec le PD
    print('alpha', alpha)


    # Implementation of PD :
    v = PD (old_distance, dist_center, KD_dist, KP_dist, dt)
    print('v', v)

    w = PD (old_angle, alpha, KD_alpha, KP_alpha, dt)
    print('w', w)

    
    print('new dist,' , dist_center, 'new angle', alpha)


    speed_r = int(BASICSPEED + v + w)
    speed_l = int(BASICSPEED + v - w)

    #if alpha > np.pi/18 or alpha < -np.pi/18:
    if alpha > np.pi/2 or alpha < -np.pi/2:
        speed_r = int(w)
        speed_l = int(- w)
    print('v2', v)


    print('speed_original,' , speed_l, speed_r)
    if speed_r > MAX_SPEED :
        speed_r = MAX_SPEED
    if speed_l > MAX_SPEED:
        speed_l = MAX_SPEED
    if dist_center <= th_dist:
        speed_l, speed_r = 0,0 

    print('speed,' , speed_l, speed_r)

    old_distance = dist_center
    old_angle = alpha

    
    return speed_l, speed_r , old_distance, old_angle


#move_to_position(4.7 , 23, 1.43, 40,40)

