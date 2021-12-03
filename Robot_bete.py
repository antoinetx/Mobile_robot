#!/usr/bin/env python
# coding: utf-8

# Implementation of "stupid" Thymios which will challenge the "clever" one
# 
# Author: Alicia Mauroux, Robotic MA1, Fall 2021
# 
# This robot have to follow a lign 
# It stops when it sees an object in front of it
# When it losts its lign, the idiot Thymio is in "lost" mode

# In[5]:


#get_ipython().system('pip3 install asgiref')


# In[2]:


# Import tdmclient Notebook environment:
import tdmclient.notebook
await tdmclient.notebook.start()


# In[3]:


from asgiref.sync import sync_to_async


# In[148]:


#test_functions = True
test_functions = False


# In[71]:


#constants
LED = 32

lost = False
w_le_old = 0 
w_ri_old = 0
sum_error_l = 0
sum_error_r = 0
l_s_old = 0 
r_s_old = 0
w_le = 0
w_ri = 0
last_turn = False #False = left; True = Right
state = 0


# Function in order to control the leds. It will tell us if the robot is doing its job or if it's lost. 

# In[19]:


@tdmclient.notebook.sync_var
def light_em_up(left=0,right=0):
    global leds_top, leds_buttons, leds_circle, lost 
    if lost:
        #red
        leds_top = [LED, 0, 0]
    else:
        #yellow
        leds_top = [LED,LED,0]
        leds_circle = [0,0,0,0,0,0,LED,0]
    if left:
        leds_circle = [0,0,0,0,0,0,LED,0]
    elif right: 
        leds_circle = [0,0,LED,0,0,0,0,0]
    else:
        leds_circle = [0,0,0,0,0,0,0,0]
        


# In[20]:


if test_functions:
    light_em_up()


# In[94]:


@onevent
def button_center():
    global state
    if button_center == 1:
        state = 1 if state==0 else 0


# In[97]:


if test_functions:
    print(state)


# In[98]:


@tdmclient.notebook.sync_var
def motors(l_speed=500, r_speed=500, verbose=False):
    """
    Sets the motor speeds of the Thymio 
    param l_speed: left motor speed
    param r_speed: right motor speed
    param verbose: whether to print status messages or not
    """
    global motor_left_target, motor_right_target
    # Printing the speeds if requested
    if verbose:
        print("\t\t Setting speed : ", l_speed, r_speed)
    motor_left_target = l_speed
    motor_right_target = r_speed
    


# In[22]:


if test_functions:
    motors(100, 100) #test with lower speed value
    sleep(2)
    motors(0, 0)


# Sensors functions

# In[53]:


@tdmclient.notebook.sync_var
def test_ground_white(white_threshold=600, verbose=False):
    """
    Tests whether the two ground sensors have seen white
    param white_threshold: threshold starting which it is considered that the sensor saw white
    param verbose: whether to print status messages or not
    """
    global turn_left, prox_ground_reflected, lost, w_le_old, w_ri_old, w_le, w_ri, last_turn
    
    
   # print(prox_ground_reflected)
    
    
    w_le = prox_ground_reflected[0] 
    w_ri = prox_ground_reflected[1] 
    #"smoothing" data
    w_le = (15*w_le + w_le_old)//16
    w_ri = (15*w_ri + w_ri_old)//16
    w_le_old = w_le
    w_ri_old = w_ri
    
    #white on the left --> turn right then to keep the track
    if (w_le > white_threshold)&(w_ri < white_threshold):
        turn_left = False
        lost = False
        last_turn = True #Last turn is to the right
        light_em_up(0,1)
        return True
    #white on the right --> turn left then to keep the track
    elif (w_le < white_threshold)&(w_ri > white_threshold):
        lost = False
        turn_left = True
        last_turn = False #Last turn is to the left
        light_em_up(1,0)
        return True
    #white on both direction --> you lost the track! turn back
    elif (w_le > white_threshold)&(w_ri > white_threshold):
        lost = True
        turn_left = False
        light_em_up()
        return True
    else:
        lost = False
        turn_left = False
        light_em_up()
        return False


# In[82]:


if test_functions:
    test_ground_white()
    print(turn_left)


# PI

# In[145]:


@tdmclient.notebook.sync_var
def PI(goal=100):
    global sum_error_l, sum_error_r, w_le, w_ri, turn_left, lost, l_s_old, r_s_old, last_turn
    error_tres = 0.1
    KP = 25
    KI = 0.1
    MAX_SPEED = 75
    MAX_ERROR = 800
    SPEED = 25
    TURN = SPEED//4
    
    if(test_object()):
        l_speed = 0
        r_speed = 0
    else:
    
        error_l = (w_le - goal)//100
        error_r = (w_ri - goal)//100
        if (abs(error_l) < error_tres):
            error_l = 0
        if (abs(error_r) < error_tres):
            error_r = 0
        if not(test_ground_white()):
            error_r = 0
            error_l = 0

        sum_error_l = sum_error_l + error_l
        sum_error_r = sum_error_r + error_r

        if (sum_error_l > MAX_ERROR):
            sum_error_l = MAX_ERROR
        elif (sum_error_l < -MAX_ERROR):
            sum_error_l = -MAX_ERROR

        if (sum_error_r > MAX_ERROR):
            sum_error_r = MAX_ERROR
        elif (sum_error_r < -MAX_ERROR):
            sum_error_r = -MAX_ERROR    

        l_speed = int(KP*error_l + KI*sum_error_l)
        r_speed = int(KP*error_r + KI*sum_error_r)

        print("left = ", l_speed)
        print("right = ", r_speed)

        if (l_speed > MAX_SPEED):
            l_speed = MAX_SPEED
        elif (l_speed < -MAX_SPEED):
            l_speed = -MAX_SPEED
        if (r_speed > MAX_SPEED):
            r_speed = MAX_SPEED
        elif (r_speed < -MAX_SPEED):
            r_speed = -MAX_SPEED

        if (test_ground_white()):
            if(turn_left):
                l_speed = SPEED - l_speed//4 - TURN
                r_speed = SPEED + r_speed//2 + TURN
            elif(lost):
                if(last_turn):
                    l_speed = MAX_SPEED
                    r_speed = -MAX_SPEED
                else:
                    l_speed = -MAX_SPEED
                    r_speed = MAX_SPEED
            else:
                l_speed = SPEED + l_speed//2  + TURN
                r_speed = SPEED - r_speed//4 - TURN
        else:
            l_speed = 2*SPEED + l_speed//2
            r_speed = 2*SPEED + r_speed//2

    l_speed = (2*l_speed + l_s_old)//3
    r_speed = (2*r_speed + r_s_old)//3
    
    l_s_old = l_speed
    r_s_old = r_speed
    
    return l_speed, r_speed


# A demander pourquoi est ce que je suis obligée d'appeler la fonction test_white avant de l'utiliser dans go_straight alors que je n'ai pas ce problème avec la fonction motor!

# In[60]:


async def go_straight(motor_speed=100, white_threshold=600, verbose=False):
    """
    Go Straight Behaviour of the FSM 
    param motor_speed: the Thymio's motor speed
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    param verbose: whether to print status messages or not
    """
    global prox_ground_reflected, test_ground_white, speed_l, speed_r
    if verbose: print("Starting go straight behaviour")
    
    l_speed, r_speed = await sync_to_async(PI)()
    
    # Move forward, i.e. set motor speeds
    motors(l_speed, r_speed)

    
    # Until one of the ground sensors sees some white
    saw_white = False
    
    
    while not saw_white:
        test_white = await sync_to_async(test_ground_white)(white_threshold, verbose=verbose)
        if test_white:
            saw_white=True
            if verbose: print("\t Saw white on the ground, exiting go straight behaviour")
        sleep(0.5) #otherwise, variables would not be updated
    return 


# In[43]:


if test_functions:
    await go_straight(100,500,True)
    motors(0, 0)


# In this function we will check is there's an obstacle on the lign the robot is following. If there's one, the robot will stop until the object is removed. 

# In[142]:


@tdmclient.notebook.sync_var
def test_object(prox_threshold=2000,prox_threshold_side=1600, verbose=True):
    """
    Tests whether the front proximity sensors saw an object on its way
    param prox_threshold: threshold starting which it is considered that the sensor saw an object
    param verbose: whether to print status messages or not
    """
    global prox_horizontal, leds_circle
    print(prox_horizontal[2], prox_horizontal[1], prox_horizontal[3])
    
    if (prox_horizontal[2]>prox_threshold):
        if verbose: print("\t\t Saw a wall")
        leds_circle = [LED,LED,LED,LED,LED,LED,LED,LED]
        return True
    elif (prox_horizontal[1]>prox_threshold_side):
        if verbose: print("\t\t Saw a wall SIDE LEFT")
        leds_circle = [0,LED,LED,LED,0,LED,LED,LED]
        return True
    elif (prox_horizontal[3]>prox_threshold_side):
        if verbose: print("\t\t Saw a wall SIDE RIGHT")
        leds_circle = [0,LED,LED,LED,0,LED,LED,LED]
        return True
    else:
        leds_circle = [0,0,0,0,0,0,0,0]
        return False


# # FSM

# In[83]:



def g_path_FSM(speed, verbose=True):
      while True:

        await go_straight(speed, verbose=verbose)
        


# In[146]:



#constants
LED = 32

lost = False
w_le_old = 0 
w_ri_old = 0
    
light_em_up()
    
g_path_FSM(10, verbose=True)


# In[147]:


#motors(0,0)


# In[ ]:




