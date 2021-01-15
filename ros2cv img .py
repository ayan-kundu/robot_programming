
import rospy
from sensor_msgs import Image
import cv2 ,cv_bridge
class Folower:

	def _init_(self):
		self.bridge=cv_bridge,CvBridge()
		cv2.namedWindow("window",1)
		self.image_sub=rospy.Subscriber("/camera/rgb/image_raw",Image,image_callback)

	def image_callback(self,msg):
		image=self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")
		cv2.imshow("window",image)
		cv2.waitKey(3)

rospy.init_node("opencv_follower")
follower=Follower()
#follower.run()
rospy.spin()