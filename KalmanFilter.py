import numpy as np

b=1

class KalmanFilter(object):
    def __init__(self, dt, point):
        self.dt=dt

        # Vecteur d'etat initial
        self.E=np.matrix([[point[0]], [point[1]], [0], [0]])

        # Matrice de transition
        self.A=np.matrix([[1, 0, self.dt, 0],
                          [0, 1, 0, self.dt],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        # Matrice d'observation, on observe que x et y
        self.H=np.matrix([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        v=1
        b=1
        self.Q=np.matrix([[v, 0, 0, 0],
                          [0, v, 0, 0],
                          [0, 0, b, 0],
                          [0, 0, 0, b]])
        
        v=1
        b=1E-5
        #v=1       
        self.R=np.matrix([[v, 0, 0, 0],
                          [0, v, 0, 0],
                          [0, 0, b, 0],
                          [0, 0, 0, b]])

        self.P=np.eye(self.A.shape[1])

    def predict(self):
        #print('les matrices prediction E sont ', self.E)
        self.E=np.dot(self.A, self.E)
       # print(' la matrice E est', self.E)
        # Calcul de la covariance de l'erreur
        self.P=np.dot(np.dot(self.A, self.P), self.A.T)+self.Q
       # print('la taille de P est', self.P.shape)
        return self.E

    def update(self, z):
       # print('la valeur  b est',b)
        # Calcul du gain de Kalman
        #print('le vecteur z est', z)
        #print('la taille de z est', z.shape)
        S=np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        K=np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
       # print('la taille de S est', S.shape)
       # print('la taille de K est', K.shape)
       # print('update 1 E', self.E)
        # Correction / innovation
        self.E=np.round(self.E+np.dot(K, (z-np.dot(self.H, self.E))))
       # print('la taille de E est', self.E.shape)
        I=np.eye(self.H.shape[1])
       # print('la taille de I est', I.shape)
        self.P=(I-(K*self.H))*self.P
        
        #print('update 2 E', self.E)
        
        return self.E
        
    def kalmanFilter(self,bool_measure, speed_l, speed_r, pos_camera, angle_robot):
        etat = self.predict().astype(np.float64)
   
        position_robot =np.array([0.0,0.0])
        position_robot[0] = etat[0][0]
        position_robot[1] = etat[1][0]
        
        if bool_measure:
            print("MESURE")
            speed = ( speed_l + speed_r)/ 2
            speed = speed / 45.045
           # print(' la vitesse moyen est', speed)

            speed_x = speed*np.cos(angle_robot)
            speed_y = speed*np.sin(angle_robot)

            if (len(pos_camera)>0):
                array = np.array([ 0,0,0,0.0])
                array[0] = float(pos_camera[0])
                array[1] = float(pos_camera[1])
                array[2] = float(speed_x)
                array[3] = float(speed_y)

                self.update(np.expand_dims(array, axis=-1))

        #print(' bool detect est',  bool_measure)
        
        #print(' les points du kF sont', etat)
        
        return  position_robot
    
    def angle_of_vectors_2(a,b,c,d):
       
        vec_a = np.array([a, b])
        vec_b = np.array([c, d])

        inner = np.inner(vec_a, vec_b)
        norms = LA.norm(vec_a) * LA.norm(vec_b)

        cos = inner / norms
        rad = np.arccos(np.clip(cos, -1.0, 1.0))

        if b < 0:
            rad = rad * (-1)

        return rad


        
