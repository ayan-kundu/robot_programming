import  rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LesarScan

def scan_call(msg):
    global range_ahead
    range_ahead=min(msg.ranges)

range_ahead=1

scan_sub=rospy.subsribe('scan',LaserScan,scan_call)
cmd_vel_pub=rospy.publish('cmd_vel',Twist,queue_size=1)

rospy.init_node('wander')
state_change_time=rospy.Time.now()
driving_forward=True

rate=rospy.Rate(1)

while not rospy.is_shutdown():

    if driving_forward:
	    if(range_ahead<2 or rospy.Time.now()>state_change_time):
 		    driving_forward=False
		    state_change_time=rospy.Time.now()+rospy.Duration(5)
    else:
        if rospy.Time.now()>state_change_time:
            driving_forward=False
            state_change_time=rospy.Time.now()+rospy.Duration(30)
    

    twist=Twist()
    if driving_forward:
        twist.linear.x=1
    else:
	    twist.angular.z=1
    
    cmd_vel_pub.publish(twist)
    
    rate.sleep()
		
