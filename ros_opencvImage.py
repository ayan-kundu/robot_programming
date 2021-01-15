
import rospy
import cv2
from std_msgs.msg  import String
#from geometry_msgs.msg import Twist
from sensor_msgs.msg        import Image
#from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
# from include.blob_detector  import *

class image_detector:
    #subscriber :robot camera topic and publisher:tele operation
    def __init__(self):
        self.bridge=CvBridge()
        cv2.namedWindow("window",1)
        self.image_sub=rospy.Subscriber("camera/rgb/image_raw",Image,self.callback)
        self.rate=rospy.Rate(3)
    def callback(self,data):
	    #ros format to cv frmt with cvBridge
        #color filtering green
        cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
		#namedWindow(" eadge")
		#gray=cv2.cvt_COLOR(cv_image,cv2.COLOR_BGR2GRAY)
		#eadge=cv2.canny(gray,100,200)
		#cv2.imshow("eadge",eadge)
        lower_green=np.array([76,100,100])
        upper_green=np.array([159,255,255])
    
        mask=cv2.inRange(hsv,lower_red,upper_red)   # 1 or 0 in range()
        res=cv2.bitwise_and(frame,frame,mask=mask)  # frame,where in the frame and mask is mask=1
        #creat a circle over the detected object
        #find its center
        
        cv2.stratWindowThread() #for high GUI image show
        cv2.imshow("image window",cv_image)
        cv2.imshow("cv window",mask)
        cv2.waitKey(3)  #wait for 3 ms to press any key|| refreshes frames per 3 ms ||if waitKey(0) it will show image


if __name__=="__main__":
    
    rospy.init_node("image_converter")
    try:
        ic=image_detector()
        #image_converter=image_detector()
        rospy.spin()
        #destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass
