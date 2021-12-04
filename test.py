import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import cv2
import numpy as np


"""

all_camera_idx_available = []

for camera_idx in range(10):
    cap = cv2.VideoCapture(camera_idx)
    if cap.isOpened():
        print(f'Camera index available: {camera_idx}')
        all_camera_idx_available.append(camera_idx)
        cap.release()
        
"""
        
VideoCap=cv2.VideoCapture(0)
VideoCap_2 = cv2.videoCapture(1)
while True:
    a=a+1
    check, frame= VideoCap.read()
    check2, frame2= VideoCap_2.read()

    cv2.imshow('image1', frame)
    cv2.imshow('image2', frame2)

    if cv2.waitKey(1)&0xFF==ord('q'):
        VideoCap.release()
        cv2.destroyAllWindows()
        break


