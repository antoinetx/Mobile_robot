# Implementation of "stupid" Thymios which will challenge the "clever" one
# Author: Alicia Mauroux, Robotic MA1, Fall 2021

#This robot have to follow a lign 
#It stops when it sees an object in front of it
#When it losts its lign, the idiot Thymio is in "alert" mode

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

#def follow_your_lign():


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

def test_ground_white(white_threshold):
    """
    Tests whether the two ground sensors have seen white
    param white_threshold: threshold starting which it is considered that the sensor saw white
    """
    if all([x>white_threshold for x in node['prox.ground.reflected']]):
        return True
    return False

async def go_straight(motor_speed=100, white_threshold=500):
    """
    Go Straight Behaviour of the FSM 
    param motor_speed: the Thymio's motor speed
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    """
    
    # Move forward, i.e. set motor speeds
    await node.set_variables(motors(motor_speed, motor_speed))
    
    # Until one of the ground sensors sees some white
    saw_white = False
    
    await node.wait_for_variables({"prox.ground.reflected"})
    
    while not saw_white:
        if test_ground_white(white_threshold):
            saw_white=True
        await client.sleep(0.2) #otherwise, variables would not be updated
    return 


def test_saw_wall(wall_threshold):
    """
    Tests whether one of the proximity sensors saw a wall
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    """
    if any([x>wall_threshold for x in node['prox.horizontal'][:-2]]):
        return True
    
    return False

async def line_following(motor_speed=30, wall_threshold=1000, white_threshold=500):
    """
    Line following behaviour of the FSM
    param motor_speed: the Thymio's motor speed
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    """
    saw_wall = False
       
    prev_state="forward"
    await node.set_variables(motors(motor_speed, motor_speed))
    
    while not saw_wall:
        if test_ground_white(white_threshold):
            if prev_state=="forward": 
                await node.set_variables(motors(motor_speed, -motor_speed))
                prev_state="turning"
        else:
            if prev_state=="turning": 
                await node.set_variables(motors(motor_speed, motor_speed))
                prev_state="forward"

        if test_saw_wall(wall_threshold): saw_wall = True
        await client.sleep(0.1) #otherwise, variables would not be updated
    return 

def test_saw_black(white_threshold):
    """
    Line following behaviour of the FSM
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    """
    
    if any([x<=white_threshold for x in node['prox.ground.reflected']]):
        return True
    
    return False


async def wall_following(motor_speed=20, wall_threshold=500, white_threshold=200):
    """
    Wall following behaviour of the FSM
    param motor_speed: the Thymio's motor speed
    param wall_threshold: threshold starting which it is considered that the sensor saw a wall
    param white_threshold: threshold starting which it is considered that the ground sensor saw white
    """
    saw_black = False
    
    await node.set_variables(motors(motor_speed, motor_speed))
           
    prev_state="forward"
    
    while not saw_black:
        
        if test_saw_wall(wall_threshold):
            if prev_state=="forward": 
                await node.set_variables(motors(motor_speed, -motor_speed))
                prev_state="turning"
        
        else:
            if prev_state=="turning": 
                await node.set_variables(motors(motor_speed, motor_speed))
                prev_state="forward"

        if test_saw_black(white_threshold): saw_black = True
        await client.sleep(0.1) #otherwise, variables would not be updated
    return 

async def g_path_FSM(speed):
    while True:
        # Step 1: line following
        await line_following(speed)
        
        # Step 2: wall following
        await wall_following(speed)
        
        # Step 3: 
        await go_straight(speed)
