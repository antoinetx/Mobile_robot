
import cv2
from Map import Map
from Detector import detect_inrange
import numpy as np

blue = 120
green = 60
ROUGE = (0, 0, 255)


marseille = Map( 1, 50)
print('hello')

print(marseille.get_map().shape)
VideoCap=cv2.VideoCapture(0)



while(True):

    ret, frame=VideoCap.read()

    points, mask=detect_inrange(frame, 800, blue)
    green_points, green_mask=detect_inrange(frame, 800, green)
    
    cv2.drawContours(frame, green_points, -1, ROUGE, 3)

    cv2.imshow('image', frame)
    
    if mask is not None:
        cv2.imshow('blue mask', mask)
        cv2.imshow('green mask', green_mask)

    if cv2.waitKey(1)&0xFF==ord('q'):
        VideoCap.release()
        cv2.destroyAllWindows()
        break



