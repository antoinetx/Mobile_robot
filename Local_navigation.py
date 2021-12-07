#!/usr/bin/env python
# coding: utf-8

# Implementation of **Local Navigation**
# 
# Author: Alicia Mauroux, Robotic MA1, Fall 2021

# Connecting to Thymio

# In[1]:




# In order to use sync_to_async

# In[2]:





# In order to do the tests:

# In[3]:

from asgiref.sync import sync_to_async
import tdmclient


# # Functions in order to test the local avoidance

# In[4]:


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



# # Local Navigation

# ### Global variables and constants

# In[5]:


LED = 32
SMALL_LED = 2

left_obstacle = False
right_obstacle = False
Bloqued = False
side = 2 #0 : Thymio turned left / 1: Thymio turned right
run = 0


# In[6]:


@tdmclient.notebook.sync_var
def light_em_up(avoid=0,right=0,wait=0):
    global leds_top, leds_buttons, leds_circle
    leds_top = [0,LED,LED]
    if(wait):
        leds_circle = [SMALL_LED, SMALL_LED, LED, SMALL_LED, SMALL_LED, SMALL_LED, LED, SMALL_LED]
    else:
        leds_circle = [0, 0, 0, 0, 0, 0, 0, 0]
    if(avoid):
        leds_top = [LED,0,LED]
    else:
        leds_top = [0,LED,LED]
        


# In[7]:


@tdmclient.notebook.sync_var
def play_music():
    global sound_system
    sound_system = [5]


# ### Check cars
# This function will check if there's something in front of Thymio. If there's, it will return **TRUE** and take the control of the Thymio. If there's nothing, it will return **FALSE** and let the control to optimal path

# In[8]:


@tdmclient.notebook.sync_var
def check_cars(state=1, Tres_high=1400, Tres_mid_side_high=1500, Tres_low=1500, Tres_mid_side_low=1000, Tres_side_high=2000):
    global prox_horizontal

 
    if((prox_horizontal[2]>Tres_high)or(prox_horizontal[1]>Tres_mid_side_high)or(prox_horizontal[3]>Tres_mid_side_high)):
        return True        
    #There's something in front of Thymio --> avoid_function take the control
    else:
        return False
    #There's nothting, Thymio continue its normal ways 


# In[9]:





# ### Avoid function
# This function will check to the left/right if there's a "Thymio-car" so our Thymio can avoid the car in front of it. If there's a "Thymio-car" in front of it and on its left/right, our Thymio will wait until the way to the left/right is free again.
# 
# **We call this function when check_cars() = True otherwise we call the logical path**

# In[11]:


@tdmclient.notebook.sync_var
def avoid_obstacle(Tres_side_high=1200, Tres_side_low=500, Tres_low=1500, Tres = 100):
    global left_obstacle, right_obstacle, prox_horizontal, sound_system
    speed0 = 100       # nominal speed
    obstSpeedGain = 5  # /100 (actual gain: 5/100=0.05)
    
    #play_music()
    
    # acquisition from the proximity sensors to detect obstacles
    obst = [prox_horizontal[0], prox_horizontal[4], prox_horizontal[2],prox_horizontal[1], prox_horizontal[3]]
    
    #check left (in order to know if Thymio is bloqued)
    if(obst[0]>Tres_side_high):
        left_obstacle = True
    elif(obst[0]<Tres_side_low):
        left_obstacle = False
        
    #check right (in order to know if Thymio is bloqued)
    if(obst[1]>Tres_side_high):
        right_obstacle = True
    elif(obst[1]<Tres_side_low):
        right_obstacle = False
    
            
    speed_l = speed0 + obstSpeedGain * int(0.9*obst[0]//100 + 0.1*obst[3]//100)
    speed_r = speed0 + obstSpeedGain * int(0.9*obst[1]//100 + 0.1*obst[4]//100)
    
    #in order to not have problems when there's just one obstacle right in front of Thymio
    if (abs(obst[0]-obst[1])<Tres):
        if obst[3]>obst[4]:
            speed_r = 0
            speed_l = speed_l + obstSpeedGain * int(0.5*obst[2]//100 + 0.5*obst[3]//100)
        else:
            speed_l = 0
            speed_r = speed_r + obstSpeedGain * int(0.5*obst[2]//100 + 0.5*obst[4]//100)

    
    #if both sides are bloqued --> wait
    if((right_obstacle)and(left_obstacle)):
        light_em_up(wait=1)
    #otherwise avoid the object:    
    else:
        light_em_up(wait=0)
        
    #in order to have a nice turn even if the side object is far away
    if right_obstacle:
        speed_l = 0
    if left_obstacle:
        speed_r = 0

    
    #If Thymio avoided the obstacle 
    if(obst[2]<Tres_low):
        if (obst[0]<Tres_side_low):
            if (obst[1]<Tres_side_low):
                light_em_up(avoid=0)
                return False
    else:
        if (speed_l > speed_r + Tres):
            speed_r = 0
        elif(speed_r > speed_l + Tres):
            speed_l = 0

    print(speed_l, speed_r)
    light_em_up(avoid=1)
    motors(speed_l,speed_r)
    return True
    
    #to do the counter later!


# ### Ideas about how to implement

# In[12]:


async def control():
    global run #bool
    
    while True:
        if run:
            run = avoid_obstacle()
        else:
            run = check_cars()
            motors(100,100)
        sleep(0.5)


# In[ ]:



    


# In[ ]:




