#! /user/bin/env pythoon

import rospy
from std_msgs.msg import String

rospy.init_node("topic_subscriber")
def callback(msg):
    print msg.data

subscriber=rospy.Subscriber('chatter',String,callback)
rospy.spin()

