'''
roscore

rostopic list
/camera/rgb/image_raw


# name python file follower.py
'''
#! usr/bin/env python
import rospy
from sensor_msgs import Image

def image_callback(msg):
	# msg.data
	pass


rospy.init_node("follower")
subsriber=rospy.Subscriber("/camera/rgb/image_raw",Image,image_callback) 

rospy.spin()

'''
chmod +x follower.py
./follower.py
rostopic hz /camera/rgb/image_raw
'''
 
