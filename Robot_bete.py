# Implementation of "stupids" Thymio which will challenge the "clever" one
# Author: Alicia Mauroux, Robotic MA1, Fall 2021

#This robot have to follow a lign 
#It stops when it sees an object in front of it
#When it losts his lign, he's in "alert" mode

start = 0 #start/stop variable --> tell us if our robot will start or if it will take a break

def start_toggle(restart):
    """
    This function will tell us if we will start or stop the robot while pushing the Start/Stop btton
    
    restart: in case the robot was lost, we can put it back on the track and restart it by pushing the Start/Stop button
    """
    if start == 0:
        start = 1
    if start == 1:
        start = 0
    if restart == 1:
        start = 1
        restart = 0

def follow_your_lign():


def motors(l_speed=500, r_speed=500):
    """
    Sets the motor speeds of the Thymio 
    param l_speed: left motor speed
    param r_speed: right motor speed
    """
    return {
        "motor.left.target": [l_speed],
        "motor.right.target": [r_speed],
    }
