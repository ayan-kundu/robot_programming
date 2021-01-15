#!/usr/bin/env python
# coding: utf-8

import cv2
import numpy as np

cap=cv2.VideoCapture(0)
while True:
    _, frame=cap.read()
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    lower_red=np.array([150,150,150])
    upper_red=np.array([180,255,255])
    
    mask=cv2.inRange(hsv,lower_red,upper_red)   # 1 or 0 in range()
    res=cv2.bitwise_and(frame,frame,mask=mask)  # frame,where in the frame and mask is mask=1
    
    cv2.imshow('frame',frame) 
    cv2.imshow('mask',mask) 
    cv2.imshow('red_filter',res)
    
    k=cv2.waitKey(5000) 
    if k== 1000:
        break
    
    
cv2.destryAllWindows()
cap.release()


