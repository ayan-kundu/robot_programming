#

What is the artifact (system) offering? (Summery of the system) :
The main purpose of this robotic system (Thorvald is the name of the robot) is to detect precisely the weeds nearby the crops and then spray herbicide in order to eradicate those to facilitate crop growth .Firstly, the robot is made auto-moveable inside the agricultural field avoiding collisions with objects as well as the walls present over there. Secondly, using RGBD-Camera mounted on its front part facing down and with the help of openCV-tools, weeds are detected and by conversion from camera coordinate to spray co-ordinate using ‘tf’ in ROS, spraying is done precisely over the weeds. 

Chosen focus area:
The chosen focus area for the artefact is mainly perception, perception over weed detection as well obstacles in front yard of the robot. The artefact involves smart obstacle avoidance algorithm also fine-tuned color filter for identifying weeds and spraying chemicals over those.

System description:
Starting with the system -
There is a python file thorvald_mover.py which by call makes Thorvald move autonomously (make sure the gazebo simulation is on and Thorvald is spawned in correctly).After that for detection part Thorvald_weed _detection.py file should be run in ROS environment (make sure roscore is in active) and now Thorvald is capable to detect the weeds nearby it and spay over the chemical on those.
The movement and obstacle avoidance of Thorvald is controlled by a simple code and obstacle avoidance algorithm. This involves laser data manipulations .A range-sensor (laser) is mounted over the robot which keeps track of reflected laser rays from the objects nearby and moves accordingly.
With help of the raw data gathered from camera and openCV tools (image processing techniques) weed is detected and each weed’s center of mass location in image frame then by camera calibration that location is projected in camera frame and after a little calculation (to find out the location of the weed in the camera frame) and use of tf in ROS the robot is enable to spray the herbicide over the detected weed successfully.

Drawbacks and settings:
With an increase in speed of the robot slight decrease accuracy in detection and spraying is observed due to lagging in the system; Linear speed is set to 2m/s and angular velocity to  XXXrad/s
Some times Thorvald runs over edges of the crops  
Spraying job is made précised as far as possible (using single mouth spray and detection software) but still there a slight chance of spray over crop leaves is left. 

Code functionality -
# import
#topics
#camera img
#opencv filtering 
#moment
#img to cam frame
#spraying





