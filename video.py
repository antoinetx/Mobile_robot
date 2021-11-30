
import cv2
from Detector import detect_inrange
import numpy as np



VideoCap=cv2.VideoCapture(0)


while(True):

    ret, frame=VideoCap.read()

    points, mask=detect_inrange(frame, 800)
    


    cv2.imshow('image', frame)
    if mask is not None:
        cv2.imshow('mask', mask)

    if cv2.waitKey(1)&0xFF==ord('q'):
        VideoCap.release()
        cv2.destroyAllWindows()
        break