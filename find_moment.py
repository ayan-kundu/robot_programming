#!/usr/bin/env python
# coding: utf-8

# In[13]:


import cv2
import numpy as np


cap=cv2.VideoCapture(0)
while True:
    _, frame=cap.read()
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) # returns 3D array 
    print(hsv.dtype,hsv.shape)
    
    # convert image to grayscale image
    #gray_image = cv2.cvtColor("C:\Users\Ayandeep Kundu\Pictures\Saved Pictures\2020-10-08.jpg", cv2.COLOR_BGR2GRAY)
    #hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_red=np.array([150,150,150])#RED FILER !
    upper_red=np.array([180,255,255])
    mask=cv2.inRange(hsv,lower_red,upper_red)   # 1 or 0 in range() #ranging filer returns b&w mask

    # calculate moments of binary image
    M = cv2.moments(mask)
    # calculate x,y coordinate of center
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    u=cx
    v=cy
    print(M["m10"],M["m00"],u,v)
    # put text and highlight the center
    cv2.circle(mask, (cx, cy), 5, (255, 0, 0), -1)# thickness=-1 to fill the circle
    #cv2.putText(img, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # display the image
    #cam = [[1, 2, 3], [5, 6,3], [7, 8, 9]]  # camera intrinsic matrix
    #x=(u-cam[0][2])/cam[0][0]
    #y=(v-cam[1][1])/cam[1][2]
    #z=1
    #print("pixel to cam co-ordinate",x,y,z)
cv2.imshow("img_moment", mask)
cv2.waitKey(0)

