
# license removed for brevity
import rospy
from std_msgs.msg import String,Int32


class Talker():
    """
    A very simple OOP talker class, publishing messages on a std_msgs/String
    topics at a fixed rate.
    """

    def __init__(self):
        """
        The "constructor" of the class, creating a Publisher and
        setting the rate
        """
        # announce the name and type of the topic we'll publish to
        # "queue_size" should always be provided, as it is the size of
        # an internal buffer for messages. Usually queue_size=1.
        # store the publisher in a member variable (self.pub), so it
        # can be used in other methods of this class (see "run")
        self.pub = rospy.Publisher('chatter', String, queue_size=1)

        # the rate to publish at
        self.rate = rospy.Rate(.3)  # .3Hz

    def run(self):
        # keep re-iterating until the node is terminated,
        # either by pressing Ctrl-C or rosnode kill command
        count=-1
        while not rospy.is_shutdown():
            count+=1
            hello_str = "%s  %d  %d " %("Hello ayan! ",rospy.get_time(),count)
            rospy.loginfo(hello_str)
            self.pub.publish(hello_str)
            self.rate.sleep()

# this is the Python way of checking that this file is run as an
# executable (like the main() function in C++ etc)
if __name__ == '__main__':
    # ALWAYS first initialise your ros node with a name,
    # optional it can have added a unique identifier (anonymous) to all
    # starting the same code several times.
    #print("Hi1")
    rospy.init_node('talker', anonymous=True)
    # "try"/"except" is the Python way of exception handling
    try:
        # instantiate the object
        #print("Hi2")
        t = Talker()  #Talker()
        # now enter the run method (will run until node terminates)
        t.run()
    except rospy.ROSInterruptException:
        # loggin a la ROS... here log the interruption as a warning
        rospy.logwarn("interrupted")
        pass
