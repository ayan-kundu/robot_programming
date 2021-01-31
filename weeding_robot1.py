#!/usr/bin/env python  # just for linux and mac based systems

# rospy for ros platform
import rospy
# for opencv tools --
import cv2
import imutils
import numpy as np
# import needful package to work with ros and python files
from cv_bridge  import CvBridge, CvBridgeError
from std_msgs.msg  import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg        import Image,CameraInfo
# interpretting images geometrically using parameters from sensor_msgs/CameraInfo
import image_geometry
# for sprayer
from std_srvs.srv import Empty
#import tf for frame transformation
import tf
# for move_base action import actionlib package
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class image_detector:
    
    def __init__(self):
        self.camera_is_ready=False

        #establish link from ros to opencv format
        self.bridge=CvBridge()
        #subscribe to thorvald camera and publish Twist msg to move thorvald
        self.image_sub=rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",Image,self.callback)
        self.cmd_vel_pub=rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel',Twist,queue_size=1)
        
        #subscribe to current camera_info topic
        self.camera_info=rospy.Subscriber("/thorvald_001/kinect2_camera/hd/camera_info",CameraInfo,self.callback_img_info)
 
        # rotate the robot 
        self.twist=Twist()

        # pinhole camera object to transform pixel coordinate to 3d vector
        self.img_geo=image_geometry.PinholeCameraModel()
        
        print("all_declared") #check point 
        
        # tf listener ----
        #get the transform From listener
        self.listener = tf.TransformListener() 

        # move_base action ----
        # Create an action client called "move_base" 
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server starts listening goals.
        self.client.wait_for_server()

        print("waiting finished ")#check point

        # sprayer service ----
        print( 'waiting for service client setup' )
        rospy.wait_for_service("thorvald_001/spray",timeout=3)               #this will thorow an error if service unavailable
        self.service_client=rospy.ServiceProxy("thorvald_001/spray",Empty)   # make this variable somewhere like __init__ func
        
        print( 'service client setup complete' )#check point


    def callback_img_info(self,msg):
        # fetching of camerainfo data  
        self.img_geo.fromCameraInfo(msg)
        # as it is only needed once 
        self.camera_info.unregister()

        self.camera_is_ready = True

    #move base action
    def movebase_client(self,area,map_coordinate):

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        if area is not None: 
            # transformation from base link to sprayer frame
            try:
                (trans,rot) = self.listener.lookupTransform('thorvald_001/base_link','thorvald_001/sprayer', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:   
                print(e)
                return
            print(trans,rot)
            # Move according the map_coordinates in world/map frame 
            goal.target_pose.pose.position.x = map_coordinate[0]+trans[0]      
            goal.target_pose.pose.position.y = map_coordinate[1]               
            goal.target_pose.pose.position.z = 0                              
            
            goal.target_pose.pose.orientation.w = 1      
            # Sends the goal to the action server.
            self.client.send_goal(goal)
            print("goal sent")
            # Waits for the server to finish performing the action.
            wait = self.client.wait_for_result( rospy.Duration(10) ) # called once
            print("done waiting for move_base")#check point

            if not wait:
                rospy.logerr("no problem! Server is unavailable (Line 104!)")
                print( 'move_base state after error: ', self.client.get_state() )
                #rospy.signal_shutdown("no prblem! Action server not available!")
            else:
                # execute the action
                print("move base running")
                return self.client.get_result() 
        else:                                        #self.cmd_vel_pub.publish(msg) # roam autoonomously
            # move to a default weed location
            goal.target_pose.pose.position.x = -0.3273 
            goal.target_pose.pose.position.y = 4.1155 
            goal.target_pose.pose.position.z = 0           #-0.2349
            goal.target_pose.pose.orientation.w = 1      
            self.client.send_goal(goal)
            print("goal sent")
            
            wait = self.client.wait_for_result( rospy.Duration(10) ) # called once
            print("done waiting ")#check point

            if not wait:
                rospy.logerr("default location not reached; Server is unavailable ")
            else:
                print("move base running to reach default location")
                return self.client.get_result() 

    # image processing method        
    def image_processing(self, data):
        # openCV format to ROS format conversion
        cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        print(" ros to cv ")#check point

		# color image to HSV conversion
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green=np.array([59,22,24])
        upper_green=np.array([110,255,255])
        # only the weeds' color would be shown as white else black
        mask=cv2.inRange(hsv,lower_green,upper_green)            # 1 or 0 in range() #ranging filer returns b&w mask
        res=cv2.bitwise_and(cv_image,cv_image,mask=mask)         # frame,where in the frame and mask is mask=1
        
        # erosion followed by dialation for on ground noise reduction and better visualisation
        kernel=np.ones((5,5),np.uint8)
        mask=cv2.erode(mask,kernel,iterations=1)
        # blur mask
        mask=cv2.GaussianBlur(mask,(11,11),cv2.BORDER_DEFAULT)
        #creat a circle over the detected object & find its center
        print("img_processed!")
        contours=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
        contours=imutils.grab_contours(contours)

        ###  CONTOUR MANIPULATION AND ROBOT MOVEMENT PART        ###
        area = None
        cx=cy=0
        print("no of contours",str(len(contours)))
        '''
        max_area=-1
        for i in (len(contours)):
            area=cv2.contourArea(contours[i])
            if area > max_area:
                c=contours[i]
                max_area=area
        print("max weed area",cv2.contourArea(c))
        max_area=cv2.contourArea(c)
        M = cv2.moments(c)
        area=max_area
        '''
        for c in contours:
            area=cv2.contourArea(c)
            weed_area=[]
            weed_area.append(area)
            max_weed_area=max(weed_area)
            #print(weed_area,"max and min area: ",max(weed_area),min(weed_area))
            print("max weed area",max_weed_area)
            # when  larger weed detected in the crop bed, run detection process 
            if area>3000: # if area >800
                # big weeds only be detected per frame
                print("detected weed area",area)
                # calculate centroid of the weed detected  using .moment()
                M = cv2.moments(c)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print('2D weed centroids coordinate',cx,cy)#check point
                    
                # draw contours
                cv2.drawContours(cv_image,[c],-1,(0,0,240),2)  
                cv2.circle(cv_image,(cx,cy),14,(0,0,240),-1)  
                # resize window
                cv_image= cv2.resize(cv_image, (730, 430))
                cv2.imshow("contour_cv_image.jpg",cv_image)
                print("contours drawn on cv_image")
                return mask,res,(cx,cy),area
            else:
                area=None
                print("No significant weed L196")
                continue    
        # return values here
        return mask,res,(cx,cy),area

    def callback(self,data):

        # check whether thorvald see any weed ; stop ,run image process and spray
        if not self.camera_is_ready:
            print(" pin hole cam is not ready")
            return

        ### run image processing   ### 
        mask,res,(cx,cy),area = self.image_processing(data)
        if area == None:
            return           

        ### pixel to 3d coordinate      ###
        print("image coordinate",(cx,cy))
        (r_x,r_y,r_z)=self.img_geo.projectPixelTo3dRay((cx,cy))
        #Scaling the coordinate wrt z=0.5 as the camera is 0.5 m above the ground
        r_x=(0.5/r_z)*r_x
        r_y=(0.5/r_z)*r_y
        r_z=0.5
        print('3D coordinate ',r_x,r_y,r_z)

        ### TRANSFORMATION of  cam ,map coordinates  by tf ###
        try:
            (trans1,rot1) = self.listener.lookupTransform('thorvald_001/kinect2_rgb_optical_frame','/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:   
            print(e)
            return
        print('tf listening')
        
        translation=trans1
        rotation=rot1
        # calculate 4*4 multiplying matrix  
        M4=self.listener.fromTranslationRotation(translation,rotation)
        # making 3d coordinate to a 4*1 matrix performing dot product
        map_coordinate=np.dot(M4,[r_x,r_y,r_z,1])
        print("weed's map_coordinate 1",map_coordinate)

        ###  movebase action server            ####  
        try:
            #result=self.movebase_client(map_coordinate)
            result=self.movebase_client(area,map_coordinate)
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")
            
        mask = cv2.resize(mask,(730, 430))         
        cv2.imshow("mask",mask)
        #res[mask>0]=[128,0,128]                                    # weeds turn into purple and purple to be sprayed... 
        res = cv2.resize(res, (730, 430))
        cv2.imshow("cv_detection_window res",res)
        print("ALL OK & ready to spray ")
        
        # check whether robot reached to desired coordinate 
        # if robot_position is not desired map_coordinate:
        #   print("no weed")   
        #   return

        # if weed size above 500 spray 
        n=0
        if area > 3000: # if area >2100
            self.service_client.call()
            n+=1
            print("sprayed %d weeds"%n)
        else:
            return        
        
        cv2.waitKey(4000) 

if __name__=="__main__":
    rospy.init_node("image_detecter")                            # node is created         
    try:
        #call to the class defined
        image_detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass
