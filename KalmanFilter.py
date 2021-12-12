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
        print('les matrices A et E sont ', self.A,self.E)
        self.E=np.dot(self.A, self.E)
        print(' la matrice E est'. self.E)
        # Calcul de la covariance de l'erreur
        self.P=np.dot(np.dot(self.A, self.P), self.A.T)+self.Q
       # print('la taille de P est', self.P.shape)
        return self.E

    def update(self, z):
        print('la valeur  b est',b)
        # Calcul du gain de Kalman
        #print('le vecteur z est', z)
        #print('la taille de z est', z.shape)
        S=np.dot(self.H, np.dot(self.P, self.H.T))+self.R
        K=np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
       # print('la taille de S est', S.shape)
       # print('la taille de K est', K.shape)

        # Correction / innovation
        self.E=np.round(self.E+np.dot(K, (z-np.dot(self.H, self.E))))
       # print('la taille de E est', self.E.shape)
        I=np.eye(self.H.shape[1])
       # print('la taille de I est', I.shape)
        self.P=(I-(K*self.H))*self.P
        

        return self.E
