import numpy as np
import cv2

class boundaryDetection():
    def getDistance(img):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # define range of red color in hsv
        lower_red = np.array([0, 50, 50],np.uint8)
        upper_red = np.array([10, 255, 255],np.uint8)
        mask1 = cv2.inRange(hsv_img, lower_red, upper_red)

        lower_red = np.array([170, 50, 50],np.uint8)
        upper_red = np.array([180, 255, 255],np.uint8)
        mask2 = cv2.inRange(hsv_img, lower_red, upper_red)

        # combine the masks
        mask = cv2.bitwise_or(mask1, mask2)
        
        # perform morphological operations
        #kernel = np.ones((5,5),np.uint8)
        #mask = cv2.erode(mask, kernel, iterations=1)
        #mask = cv2.dilate(mask, kernel, iterations=2)
        #cv2.imwrite('test1.jpg', mask)
        
        edges = cv2.Canny(mask, 50, 150)
        
        # detect line segments using HoughLinesP
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=5, maxLineGap=10)
        
        center_x = int(img.shape[1]/2)
        center_y = int(img.shape[0]/2)
        distance =0
        count = 0
        midX = 0
        midY = 0
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line[0]
                if abs(y2-y1)>=0: # filter horizontal lines
                    cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
                    mid_x = int((x1+x2)/2)
                    mid_y = int((y1+y2)/2)
                    count = count + 1
                    midX = midX + mid_x
                    midY = midY + mid_y
                    cv2.circle(img, (mid_x, mid_y), 5, (0,0,255), -1)
        elif count == 0:
            return -1
                    
        cv2.circle(img, (center_x, center_y), 5, (255,0,0), -1)
            
        midX = midX/count
        midY = midY/count
        cv2.circle(img, (int(midX), int(midY)), 5, (255,255,255), -1)
        distance = np.sqrt(pow((midX-center_x),2) + pow((midY-center_y),2))
        
        return distance
        