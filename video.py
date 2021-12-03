
import cv2
from Map import Map
from Detector import detect_inrange
from KalmanFilter import KalmanFilter
import numpy as np

blue = 120
green = 60
ROUGE = (0, 0, 255)


marseille = Map( 1, 50)
print('hello')

print(marseille.get_map().shape)
VideoCap=cv2.VideoCapture(0)

KF=KalmanFilter(0.1, [0, 0])

while(True):

    ret, frame=VideoCap.read()

    points, mask=detect_inrange(frame, 800, blue)
    green_points, green_mask=detect_inrange(frame, 800, green)
    
    etat=KF.predict().astype(np.int32)
    cv2.circle(frame, (int(etat[0]), int(etat[1])), 2, (0, 255, 0), 5)
    
    cv2.arrowedLine(frame, (int(etat[0]), int(etat[1])), (int(etat[0]+etat[2]), int(etat[1]+etat[3])),
                        color=(0, 255, 0), thickness=3)
    
    #cv2.drawContours(frame, green_points, -1, ROUGE, 3)

    cv2.imshow('image', frame)
    
    if (len(points)>0):
        cv2.circle(frame, (points[0][0], points[0][1]), 10, (0, 0, 255), 2)
    
    if mask is not None:
        cv2.imshow('blue mask', mask)
        cv2.imshow('green mask', green_mask)

    if cv2.waitKey(1)&0xFF==ord('q'):
        VideoCap.release()
        cv2.destroyAllWindows()
        break



