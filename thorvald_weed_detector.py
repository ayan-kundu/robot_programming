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
        #establish link from ros to opencv format
        self.bridge=CvBridge()
        #subscribe to thorvald camera and publish Twist msg to move thorvald
        self.image_sub=rospy.Subscriber("/thorvald_001/kinect2_camera/hd/image_color_rect",Image,self.callback)
        self.cmd_vel_pub=rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel',Twist,queue_size=1)
        #subscribe to current camera_info topic
        self.camera_info=rospy.Subscriber("/thorvald_001/kinect2_camera/hd/camera_info",CameraInfo,self.callback_img_info)
 
        # rotate the robot 
        self.twist=Twist()
        # every 1 s it will be refreshed
        self.rate=rospy.Rate(1)
        # pinhole camera object to transform pixel coordinate to 3d coordinate wrt camera frame
        self.img_geo=image_geometry.PinholeCameraModel()
        
        print("all_declared") #check point 
        
        # tf listener ----
        #get the transform From listener
        self.listener = tf.TransformListener() 

        # move_base action ----
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        print("waiting finished ")

        # sprayer service ----
        rospy.wait_for_service("thorvald_001/spray",timeout=3) #this will thorow an error if service unavailable
        self.service_client=rospy.ServiceProxy("thorvald_001/spray",Empty)   # make this variable somewhere like __init__ func


    def callback_img_info(self,msg):
        # fetching of camerainfo data  
        self.img_geo.fromCameraInfo(msg)
        # as it is only needed once 
        self.camera_info.unregister()

    #move base action
    
    def movebase_client(self,map_coordinate):
        
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move according the map_coordinates in world/map frame 
        goal.target_pose.pose.position.x = map_coordinate[0] #-0.46311742 
        goal.target_pose.pose.position.y = map_coordinate[1] #-4.17777844 
        goal.target_pose.pose.position.z = 0                 #  0.06975775
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1              #  1.

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        print("goal sent")
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        print("done waiting")
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("no problem! Server is unavailable (Line 90!)")
            rospy.signal_shutdown("no prblem! Action server not available!")
        else:
            # Result of executing the action
            return self.client.get_result() 

    def callback(self,data):
        # IMAGE PROCESSING PART :
        #'''
	    # ros format to cv frmt with cvBridge
        cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        #cv2.imshow("image window",cv_image)
        print(" ros to cv ")
		# color image to HSV conversion
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # color filtering with green channel mostly close to weeds' color
        #lower_green=np.array([60,20,20])
        #upper_green=np.array([110,255,255])
        lower_green=np.array([61,22,28])
        upper_green=np.array([111,255,255])
        # only the weeds' color would be shown as white else black
        mask=cv2.inRange(hsv,lower_green,upper_green)   # 1 or 0 in range() #ranging filer returns b&w mask
        res=cv2.bitwise_and(cv_image,cv_image,mask=mask)      # frame,where in the frame and mask is mask=1
        #creat a circle over the detected object
        #find its center
        print("img_processed!")
        # erosion followed by dialation for on ground noise reduction and better visualisation
        kernel=np.ones((5,5),np.uint8)
        res=cv2.erode(res,kernel,iterations=1)
        res=cv2.dilate(res,kernel,iterations=1) 
        mask=cv2.erode(mask,kernel,iterations=1)
        mask=cv2.dilate(mask,kernel,iterations=1)
        print("mask",mask)
        # cnts tracks the contours found in the images
        cnts0=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #print("cnts0",cnts0,np.shape(cnts0)) # returns array of (3, )
        cnts=imutils.grab_contours(cnts0)
        #print("cnts0",cnts0,np.shape(cnts))
        
        # loop to manipulate each found contours/weeds and find its centroid
        # centroid is assumed as the location of weed
        '''
        '''
        # CONTOUR MANIPULATION AND ROBOT MOVEMENT PART :
        for c in cnts:
            area=cv2.contourArea(c)
            '''
            #if weed detected
            while area:
                weed_area=[]
                weed_area.append(area)
                #print(weed_area,"max and min area: ",max(weed_area),min(weed_area))
            '''
            if area<500:
                continue

            # big weeds only be detected per frame
            #while area>300 and area<900:
            print("detected weed area",area)
            # finding centroids of the white regions in mask
            M = cv2.moments(c)
            # calculate centroid of the weed detected  using .moment()
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            print('2D weed centroids coordinate',cx,cy)
                
            # highlight the center
            cv2.drawContours(cv_image,[c],-1,(0,0,255),3)   #cv2.drawContours(res,[c],-1,(0,255,0),3)
            cv2.circle(cv_image,(cx,cy),7,(255,255,255),-1)  
            #resize contoured windows and show
            cv_image= cv2.resize(cv_image, (930, 530))
            cv2.imshow("contour_cv_image",cv_image)
            print("contours drawn on cv_image")

            #pixel to 3d coordinate
            (r_x,r_y,r_z)=self.img_geo.projectPixelTo3dRay((cx,cy))
            print('3D coordinate ',r_x,r_y,r_z)

            ### TRANSFORMATION of  coordinates  by tf ##############
            try:
                (trans,rot) = self.listener.lookupTransform('/thorvald_001/kinect2_link','/map', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:   
                print(e)
                continue
            print('tf listening')

            translation=trans
            rotation=rot
            #calculate 4*4 matrix via listener.fromTranslationRotation(self,translation,rotation)
            M4=self.listener.fromTranslationRotation(translation,rotation)
            print("M 4x4",M4)
            # making 3d coordinate 4*1 matrix and perform dot product
            map_coordinate=np.dot(M4,[r_x,r_y,r_z,1])# 4x1 array as result
            print("weed's map_coordinate",map_coordinate)
            '''
            #  movebase action server problem 
            try:
                #result=self.movebase_client(map_coordinate)
                result=self.movebase_client()
                if result:
                    rospy.loginfo("Goal execution done!")
            except rospy.ROSInterruptException:
                rospy.loginfo("Navigation test finished.")
            '''
                
            break # getting out for loop

        # update frame and cnts # frame updates as robot moves forward
        '''
        '''    
        # shows image windows in screen
        #cv_image= cv2.resize(cv_image, (930, 530))
        #cv2.imshow("original_image",cv_image)
        mask = cv2.resize(mask, (930, 530))
        cv2.imshow("mask_image",mask)
        #res[mask>0]=[0,0,235]  #weeds turn into purple and purple to be sprayed... 
        res = cv2.resize(res, (930, 530))
        #cv2.imshow("cv_detection_window",res)
        print("ALL OK")
        
        #refresh every 1 s
        #self.rate.sleep()

        #call sprayer to spray ...
        # most of the weed size below 150 then only it will spray 
        if area>400 and area<900:
            self.service_client.call() # Run to spray
            print("sprayed")
        else:
            pass
        
        cv2.waitKey(4000)  #wait for 1000 ms to press any key|| refreshes frames per 3 ms ||if waitKey(0) it will show image
        
        #break  # getting out of callback function
        #'''
if __name__=="__main__":
    
    rospy.init_node("image_detecter")
    try:
        #call to the class defined
        image_detector()
        #######################################
        rospy.spin()
        #destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.logwarn("interrupted")
        pass
