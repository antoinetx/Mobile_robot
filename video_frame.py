import cv2

def frame_capture(take_frame_nb, camera):
    """
    Take a video frame from the choosen camera
    :param take_frame_nb: number of frame not taken before the good frame (to make sure to have a good image)  
    :param camera: choose the camera : 0 webcam, 1 extern camera)
    :Return: the video frame
    """
    
    VideoCap=cv2.VideoCapture(camera)
    VideoCap.isOpened()
    #print(return_camera_indices())
    for i in range(take_frame_nb):
        check, frame= VideoCap.read()
        cv2.imshow('image', frame)
    VideoCap.release()
    cv2.destroyAllWindows()
    return frame
