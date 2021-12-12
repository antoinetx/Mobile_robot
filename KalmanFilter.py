# Implementation of the  *Kalman Filter*
# 
# Origin file: https://github.com/L42Project/Tutoriels/tree/master/Divers/tutoriel36 from the youtube chanel L42Project
# 
# Modified and adapted by Xavier Nal, Robotic MA1, Fall 2021
#
#
##

# imports

import numpy as np
import numpy.linalg as LA

RATIO_SPEED = 45.045


class KalmanFilter(object):
    def __init__(self, dt, point):
        """ Initialisation of the matrix for the Kalman Filter """
        self.dt=dt

        # State vector
        self.E=np.matrix([[point[0]], [point[1]], [0], [0]])

        # Transition matrix
        self.A=np.matrix([[1, 0, self.dt, 0],
                          [0, 1, 0, self.dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        # Observation matrix
        self.H=np.matrix([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        
        #covariances matrix
        v=1
        b=1
        self.Q=np.matrix([[v, 0, 0, 0],
                          [0, v, 0, 0],
                          [0, 0, b, 0],
                          [0, 0, 0, b]])
        
        v=1
        b=1E-5     
        self.R=np.matrix([[v, 0, 0, 0],
                          [0, v, 0, 0],
                          [0, 0, b, 0],
                          [0, 0, 0, b]])

        self.P=np.eye(self.A.shape[1])
        

    def predict(self):
        """ Predict step """

        self.E=np.dot(self.A, self.E)
 
        # Calculation of the covariance of the error
        self.P=np.dot(np.dot(self.A, self.P), self.A.T)+self.Q
 
        return self.E
    

    def update(self, z):
        """ Update step of the kalman filter  """
         
        # Compute the Kalman gain
        S=np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        K=np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
  
        # Correction / innovation
        self.E=np.round(self.E+np.dot(K, (z-np.dot(self.H, self.E))))
        I=np.eye(self.H.shape[1])
        self.P=(I-(K*self.H))*self.P
        
        return self.E
    
        
    def kalmanFilter(self,bool_measure, speed_l, speed_r, pos_camera, angle_robot):
        """ Extern function which take in input the speed of the two wheel, the orientation of the robot
         and the bool which inform about the measurement"""
         
        # call the prediction step of the kalman filter
        etat = self.predict().astype(np.float64)
    
        position_robot =np.array([0.0,0.0])
        position_robot[0] = etat[0][0]
        position_robot[1] = etat[1][0]
        
        if bool_measure:
            speed = ( speed_l + speed_r)/ 2
            speed = speed / RATIO_SPEED

            speed_x = speed*np.cos(angle_robot)
            speed_y = speed*np.sin(angle_robot)

            # call the update function
            if (len(pos_camera)>0):
                array = np.array([ 0,0,0,0.0])
                array[0] = float(pos_camera[0])
                array[1] = float(pos_camera[1])
                array[2] = float(speed_x)
                array[3] = float(speed_y)

                self.update(np.expand_dims(array, axis=-1))
        
        return  position_robot
    
    
    def angle_of_vectors_2(self,a,b,c,d):
        """ Maths function to calcul the angle between to vector """
       
       # if one vector is the zero vector
        if((a == 0) and (b == 0)):
            return 0
        
        vec_a = np.array([a, b])
        vec_b = np.array([c, d])
        
        inner = np.inner(vec_a, vec_b)
        norms = LA.norm(vec_a) * LA.norm(vec_b)

        cos = inner / norms
        rad = np.arccos(np.clip(cos, -1.0, 1.0))

        if b < 0:
            rad = rad * (-1)

        return rad
