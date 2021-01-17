#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg  import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg        import Image
#from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
# from include.blob_detector  import *
#import colorsys


class image_detector:
    #subscriber :robot camera topic and publisher:tele operation
    def __init__(self):
        self.bridge=CvBridge()
	
        self.image_sub=rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",Image,self.callback)
        self.cmd_vel_pub=rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel',Twist,queue_size=1)
        
	self.twist=Twist()
        print("all_declared")
    def callback(self,data):
	#ros format to cv frmt with cvBridge
        #color filtering green
        
        cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")

        #cv2.imshow("image window",cv_image)
        print("CB ok")
	#
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green=np.array([60,20,20])
        upper_green=np.array([110,255,255])
        #
	mask=cv2.inRange(hsv,lower_green,upper_green)   # 1 or 0 in range() #ranging filer returns b&w mask
        res=cv2.bitwise_and(cv_image,cv_image,mask=mask)      # frame,where in the frame and mask is mask=1
        #creat a circle over the detected object
        #find its center
        print("img_processed")
        #
        #
        #h,w,d=cv_image.shape
        #search_top=3*h/4
        #search_bottom=search_top +20
        #mask[0:search_top,0:w]=0
        #mask[search_bottom:0,0:w]=0
        # M=cv2.moment(mask)    # finding  mask moment info
        # #print("moment cal")
        # #finding err b/w circle center and cv_image center over x axis
        # #rotate as to the value of err
        # if M['m00']>0:
        #     cx=int(M['m10']/M['m00'])
        #     cy=int(M['m01']/M['m00'])
        #     cv2.circle(cv_image,(cx,cy),20,(0,0,255),-1)
        #     err=cx-w/2
        #     self.twist.linear.x=0.2
        #     self.twist.angular.z=-float(err)/100
        #     self.cmv_vel_pub(self.twist)  # published cmd_vel_pub
        #print("weed position followd")
        #cv2.startWindowThread()
        #
        #resize image window
        #print('cv_image Dimensions : ',cv_img.shape)
        scale_percent = 60 # percent of original size
        width = int(cv_img.shape[1] * scale_percent / 100)
        height = int(cv_img.shape[0] * scale_percent / 100)
        dim = (width, height)
        resized = cv2.resize(cv_img, dim, interpolation = cv2.INTER_AREA)
	#cv2.namedWindow("output", cv2.WINDOW_NORMAL)  
        #res = cv2.resize(res, (960, 540))
        #print('Resized Dimensions : ',resized.shape)
        cv2.imshow("Resized image", resized)
        cv2.imshow("image window",cv_image)
        #cv2.imshow("cv window",mask)
        #res[mask>0]=[0,0,235]  #weeds turn into purple and purple to be sprayed...
        cv2.imshow("cv window",res)
	# finding moment of detected weed in mask
	M = cv2.moments(mask)
    	# calculate x,y coordinate of center
    	cx = int(M["m10"] / M["m00"])
    	cy = int(M["m01"] / M["m00"])
    	u=cx
    	v=cy
    	#print(M["m10"],M["m00"],u,v)
    	cv2.circle(mask, (cx, cy), 5, (255, 0, 0), -1)# thickness=-1 to fill the circle
    	# display the image
    	#cam = [[1, 2, 3], [5, 6,3], [7, 8, 9]]  # camera intrinsic matrix
    	#x=(u-cam[0][2])/cam[0][0]
    	#y=(v-cam[1][1])/cam[1][2]
    	#z=1
    	#print("pixel to cam co-ordinate",x,y,z)
	
        cv2.waitKey(3)  #wait for 3 ms to press any key|| refreshes frames per 3 ms ||if waitKey(0) it will show image
        #print("done")
if __name__=="__main__":
    
    rospy.init_node("image_detecter")
    try:
        image_detector()
        #image_converter=image_detector()
        #ic.run()         /   image_converter.run()
        rospy.spin()
        #destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass
