import numpy as np
import cv2

blue = 120
green = 60
sensitivity = 20

blue_lo=np.array([100, 50, 50])
blue_hi=np.array([140, 255, 255])

green_lo=np.array([40, 50, 50])
green_hi=np.array([80, 255, 255])

def detect_inrange(image, surface, color):
    points=[]
    lo=np.array([color - sensitivity, 50, 50])
    hi=np.array([color + sensitivity, 255, 255])

    image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image=cv2.blur(image, (5, 5))
    mask=cv2.inRange(image, lo, hi)
    mask=cv2.erode(mask, None, iterations=2)
    mask=cv2.dilate(mask, None, iterations=2)
    elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    elements=sorted(elements, key=lambda x:cv2.contourArea(x), reverse=True)
    
    for element in elements:
        if cv2.contourArea(element)>surface:
            ((x, y), rayon)=cv2.minEnclosingCircle(element)
            points.append(np.array([int(x), int(y)]))
        else:
            break

    return points, mask, elements

def detect_center(image, contours):
    a=0
    center_points=[]
    center_contours=[]
    
    for i in contours:
        M = cv2.moments(i)
        if M['m00'] != 0:
            #print('valeur')
            #print(a)
            a += 1
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            center_points.append(np.array([int(cx), int(cy)]))
            center_contours.append(i)
            """
            cv2.drawContours(image, [i], -1, (0, 255, 0), 2)
            cv2.circle(image, (cx, cy), 7, (0, 0, 255), -1)
            cv2.putText(image, "center", (cx - 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    """
    #print(f"x: {cx} y: {cy}")
    return center_points, center_contours

def detect_obstacle(image_filtered):
    reduction = 10
    print(image_filtered.shape)
    map = np.zeros((64,64))
    print(np.unique(map))
    
    #if image_filtered[(image_filtered == 255 )]:
    #    map[image_filtered[(image_filtered == 255 )]//10] = 1
        
    result = np.where(image_filtered == np.amax(image_filtered))
    
    print('List of coordinates of maximum value in Numpy array : ')
    # zip the 2 arrays to get the exact coordinates
    listOfCordinates = list(zip(result[0], result[1]))
    
    # travese over the list of cordinates
    i=0
    for cord in listOfCordinates:
        i +=1;
        if i < 10:
            print(64 - int(cord[0]/reduction),  int(cord[1]/reduction))
            print(64 - int(cord[0]),  int(cord[1]))
        map[64 - int(cord[0]/reduction),  int(cord[1]/reduction) ] = 1
        
    
    #print(np.where(image_filtered == 255 ))
    #print(map)
    
    
    print('La map filtree')
    
    print(np.unique(map))
    print(map.shape)
    
    return map
        
    
    """
    if (len(points)>0):
        for i in range(len(points)):
            x = int(480-points[i][1])
            y = points[i][0]
            map[x, y] = 1
            occupancy_grid[int(x/reduction),int(y/reduction)]=1
            print('maison')
            for v in range(int(x/reduction),int(x/reduction +10)):
                for w in range(int(y/reduction),int(y/reduction +10)):
                    if (v-5) <48 or  (w-5)<64:
                        if occupancy_grid[int(v-5),int(w-5)] != 2:
                            occupancy_grid[int(v-5),int(w-5)]=1
            
            print(points[i][0], points[i][1])
            
    """
    
