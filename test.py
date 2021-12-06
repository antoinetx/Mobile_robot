
import cv2
import numpy as np

       
        
VideoCap=cv2.VideoCapture(0)
check, frame= VideoCap.read()

print(frame.shape)
#print(return_camera_indices())

while True:
    
    check, frame= VideoCap.read()
    

    cv2.imshow('image', frame)
    

    if cv2.waitKey(1)&0xFF==ord('q'):
        VideoCap.release()
        cv2.destroyAllWindows()
        break








    """
    
    def return_camera_indices():
        index = -2
        arr = []
        i = 10
        while i > 0:
            cap = cv2.VideoCapture(index)
            if cap.read()[0]:
                arr.append(index)
                cap.release()
            index += 1
            i -= 1
        return arr
    """