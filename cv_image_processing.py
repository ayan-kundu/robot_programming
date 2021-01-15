
import rospy
import cv2
#from std_msgs.msg  import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg        import Image
from cv_bridge              import CvBridge, CvBridgeError
# from include.blob_detector  import *

class image_detector:
    #subscriber :robot camera topic and publisher:tele operation
    def __init__(self):
        self.bridge=CvBridge()
        cv2.namedWindow("window",1)
        #self.image_sub=rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)
        #self.image_pub=rospy.Publisher('cmd_vel_mux/input/teleop',Twist,queue_size=1)
        self.twist=Twist()
        self.rate=rospy.Rate(3) #3 hz
    def callback(self,data):
	    
        
        #ros format to cv frmt with cvBridge
        #color filtering green
        cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
		
        lower_green=np.array([40,10,10])   # thorvald world simulation
        upper_green=np.array([110,255,255])  # HSV parameters
    
        mask=cv2.inRange(hsv,lower_red,upper_red)   # 1 or 0 in range()
        res=cv2.bitwise_and(frame,frame,mask=mask)  # frame,where in the frame and mask is mask=1
        
        kernel=np.ones((5,5),np.uint8)
        opening=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        closing=cv2.morphEx(mask,cv2.MORPH_CLOSE,kernel)

        cv2.imshow("Opening",opening)
        cv2.imshow("Closing",closing)

        
        #creat a circle over the detected object
        #find its center

        h,w,d=cv_image.shape
        search_top=3*h/4
        search_bottom=search_top +20
        mask[0:search_top,0:w]=0
        mask[search_bottom:0,0:w]=0
        M=cv2.moment(mask)    # finding  mask moment info
        #finding err b/w circle center and cv_image center over x axis
        #rotate as to the value of err
        if M['m00']>0:
            cx=int(M['m10']/M['m00'])
            cy=int(M['m01']/M['m00'])
            cv2.circle(cv_image,(cx,cy),20,(0,0,255),-1)
            err=cx-w/2
            self.twist.linear.x=0.2
            self.twist.angular.z=-float(err)/100
            self.cmv_vel_pub(self.twist)

#option for control direction of robot
#with output from opencv





#show cv output

        cv2.stratWindowThread() #for high GUI image show
        cv2.imshow("image window",cv_image)
        cv2.imshow("cv window",mask)
        cv2.waitKey(3)  #wait for 3 ms to press any key|| refreshes frames per 3 ms ||if waitKey(0) it will show image


if __name__=="__main__":
    
    rospy.init_node("image_converter")
    try:
        ic=image_detector()
        #image_converter=image_detector()
        #ic.run()         /   image_converter.run()
        rospy.spin()
        #destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass

