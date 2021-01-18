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
import colorsys


class image_detector:
    #subscriber :robot camera topic and publisher:tele operation
    def __init__(self):
        self.bridge=CvBridge()
        #cv2.namedWindow("storing_window",1)
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
		#namedWindow(" eadge")
		#gray=cv2.cvt_COLOR(cv_image,cv2.COLOR_BGR2GRAY)
		#eadge=cv2.canny(gray,100,200)
		#cv2.imshow("eadge",eadge)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green=np.array([60,20,20])
        upper_green=np.array([110,255,255])
        mask=cv2.inRange(hsv,lower_green,upper_green)   # 1 or 0 in range()
        res=cv2.bitwise_and(cv_image,cv_image,mask=mask)      # frame,where in the frame and mask is mask=1
        #creat a circle over the detected object
        #find its center
        print("img_processed")
        h,w,d=cv_image.shape
        search_top=3*h/4
        search_bottom=search_top +20
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
        cv2.imshow("image window",cv_image)
        cv2.imshow("cv window",mask)
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

