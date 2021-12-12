#!/usr/bin/env python
# coding: utf-8

# Implementation of **Local Navigation**
# 
# Author: Alicia Mauroux, Robotic MA1, Fall 2021

from asgiref.sync import sync_to_async
import tdmclient

# Global variables and constants
WAIT = 0
I_c = 0.9 #Coefficient about the side horizontal sensors in the "interior"
E_c = 0.1 #Coefficient about the side horizontal sensors in the "exterior"
CENT = 100
HALF = 0.5
KEEP_GOING = 30
INIT = 0

# Variables to detect the obstacles and avoid them

PROX_FRONT = 2000
PROX_COTE1 = 2700
PROX_COTE_BORD = 2000

# Varaiables to know that the obstacle is avoided


left_obstacle = False
right_obstacle = False
compt = WAIT
conti = KEEP_GOING
##################
# MAIN FUNCTIONS
##################

        
# ### Check cars
# This function will check if there's something in front of Thymio. If there's, it will return **TRUE** and take the control of the Thymio. If there's nothing, it will return **FALSE** and let the control to optimal path
@tdmclient.notebook.sync_var
def check_cars(Tres_high=PROX_FRONT, Tres_mid_side_high=PROX_COTE1, Tres_side_high= PROX_COTE_BORD, prox_horizonta=INIT):

    if((prox_horizonta[2]>Tres_high)or(prox_horizonta[1]>Tres_mid_side_high)or(prox_horizonta[3]>Tres_mid_side_high)):
        return True        
    #There's something in front of Thymio --> avoid_function take the control
    else:
        return False
    #There's nothting, Thymio continue its normal ways 


# ### Avoid function
# This function will check to the left/right if there's a "Thymio-car" so our Thymio can avoid the car in front of it. If there's a "Thymio-car" in front of it and on its left/right, our Thymio will wait until the way to the left/right is free again.
# 
# **We call this function when check_cars() = True otherwise we call the logical path**

@tdmclient.notebook.sync_var
def avoid_obstacle( Tres_side_high=1200, Tres_side_low=1000, Tres_low=1500, Tres = 500, prox_horizonta=0):
    global left_obstacle, right_obstacle, compt, conti
    speed0 = 70       # nominal speed
    obstSpeedGain = 5  # /100 (actual gain: 5/100=0.05)

    
    # acquisition from the proximity sensors to detect obstacles
    obst = [prox_horizonta[0], prox_horizonta[4], prox_horizonta[2],prox_horizonta[1], prox_horizonta[3]]
    
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
    
            
    speed_l = speed0 + obstSpeedGain * int(I_c*obst[0]//CENT + E_c*obst[3]//CENT)
    speed_r = speed0 + obstSpeedGain * int(I_c*obst[1]//CENT + E_c*obst[4]//CENT)
    
    #in order to not have problems when there's just one obstacle right in front of Thymio
    if (abs(obst[0]-obst[1])<Tres):
        if obst[3]>obst[4]:
            speed_l = speed_l + obstSpeedGain * int(HALF*obst[2]//CENT + HALF*obst[3]//CENT)
            speed_r = - speed_l
        else:
            speed_r = speed_r + obstSpeedGain * int(HALF*obst[2]//CENT + HALF*obst[4]//CENT)
            speed_l = - speed_r

        
    #in order to have a nice turn even if the side object is far away
    if right_obstacle:
        speed_l = -speed_r
    if left_obstacle:
        speed_r = -speed_l
    #if Thymio is bloqued -> it turns away
    if(right_obstacle)and(left_obstacle):
        speed_l = speed0
        speed_r = -speed0

    
    #If Thymio avoided the obstacle 
    if(obst[2]<Tres_low):
        print('dans la boucle 1')
        if (obst[3]<Tres_side_low):
            print('dans la boucle 2')
            if (obst[4]<Tres_side_low):
                print('dans la boucle 3')
                #wait until the other car went away
                if conti >0 :
                    conti = conti - 1
                    speed_l = speed0
                    speed_r = speed0
                    return speed_l, speed_r, True
                
                if (compt > 0):
                    compt = compt - 1 
                    speed_l = 0
                    speed_r = 0
                    print('compteur' , compt)
                    return speed_l, speed_r, True

                compt = WAIT
                conti = KEEP_GOING
                return speed_l, speed_r, False
                                  
    else:
        if (speed_l > speed_r + Tres):
            speed_r = -speed_l
        elif(speed_r > speed_l + Tres):
            speed_l = -speed_r
    return speed_l, speed_r, True