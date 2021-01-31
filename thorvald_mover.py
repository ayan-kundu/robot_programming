#!/usr/bin/env python
import rospy ,time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover:
    """
    A very simple Roamer implementation for Thorvald.
    It simply goes straight until any obstacle is within
    3 m distance and then just simply turns left.
    A purely reactive approach.
    """

    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        self.publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
        rospy.Subscriber("/thorvald_001/scan", LaserScan, self.callback)
        self.rate=rospy.Rate(.1)
    def callback(self, data):
        """
        Callback called any time a new laser scan becomes available
        """

        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.seq)
        rospy.loginfo("Robot moving - %s"%rospy.get_time)
        #self.rate.sleep()
        min_dist = min(data.ranges)
        t = Twist()
        if min_dist < 1.2:   #within 2 m distance
            #t.linear.x=-.7
            #time.sleep(.01)
            t.angular.z = 0.9
        else:
            t.linear.x = .2  #2.1
            self.publisher.publish(t)
        
        #self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mover')
    Mover()
    rospy.spin()

