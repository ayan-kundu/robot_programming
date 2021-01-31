# Thorvald Weed spray
**Version 1.0.0**

*An autonomous robot names Thorvald is introdused and developed at the University of Lincoln on collaboration with the robotics company SAGA Ltd to build a system that can accomplish a defined task (see below)*

## What is the artifact (system) offering? :
> (Summery of the system) 

* The main purpose of this robotic system (Thorvald is the name of the robot) is to detect precisely the weeds nearby the crops and then spray herbicide in order to eradicate those to facilitate crop growth.
* Firstly, the robot is made auto-moveable inside the agricultural field avoiding collisions with objects as well as the walls present over there.
* Secondly, using RGBD-Camera mounted on its front part facing down and with the help of openCV-tools, weeds are detected and by conversion from camera coordinate to spray co-ordinate using ‘tf’ in ROS, spraying is done precisely over the weeds. 

## Chosen focus area:
* The chosen focus area for the artefact is mainly perception, perception over weed detection as well obstacles in front yard of the robot. The artefact involves smart obstacle avoidance techniques also fine-tuned color filter and image processing for identifying weeds and spraying chemicals over them.

## System description:
> *Starting with the system -*

1. There is a python file thorvald_mover.py which by call makes Thorvald move autonomously (make sure the gazebo simulation is on and Thorvald is spawned in correctly).After that for detection part Weeding_robot1.py file should be run in ROS environment (make sure roscore is in active) and now Thorvald is capable to detect the weeds nearby it and spay over the chemical on those.
2. The movement and obstacle avoidance of Thorvald is controlled by a simple code and obstacle avoidance algorithm. This involves laser data manipulations .A range-sensor (laser) is mounted over the robot which keeps track of reflected laser rays from the objects nearby and moves accordingly.
3. With help of the raw data gathered from camera and openCV tools (image processing techniques) weed is detected and each weed’s center of mass location in image frame then by camera calibration that location is projected in camera frame and after a little calculation (to find out the location of the weed in the camera frame) and use of tf in ROS the robot is enable to spray the herbicide over the detected weed successfully.

#### Drawbacks and settings:
1. With an increase in speed of the robot slight decrease accuracy in detection and spraying is observed due to lagging in the system; Linear speed is set to .2 m/s and angular velocity to  -0.9 rad/s.
2. Some times Thorvald runs over edges of the crops(although it is rare).  
3. Spraying job is made précised as far as possible (using single mouth spray and detection software) but still there a slight chance of spray over crop leaves and missing spray over weeds is left. 
4. although weed is detected and filtered fine but due to poor manipulation over their centroid tracking the system becomes a bit poor performing; therefore the spraying job is disturbed.
##### Code functionality and description -

> launch [thorvald-sim.launch](https://github.com/LCAS/CMP9767M/tree/master/uol_cmp9767m_base/launch) to work with thorvald simulation in gazebo
----
> launch [move_base.launch](https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/launch/move_base.launch) to use move_base action called inside the main code-weeding_robot1.py
----
To use this launch files one has to have related files in [config](https://github.com/LCAS/CMP9767M/tree/master/uol_cmp9767m_tutorial) and [launch](https://github.com/LCAS/CMP9767M/tree/master/uol_cmp9767m_base) folder placed in [LCAS/CMP9767M](https://github.com/LCAS/CMP9767M)
----
- [thorvald_mover.py](https://github.com/ayan-kundu/robot_programming/blob/main/thorvald_mover.py)
> Make sure you are not running move_base.launch and thorvald_mover at a time otherwise it can creat confusion for the robot to move.
----
- [weeding_robot1.py](https://github.com/ayan-kundu/robot_programming/blob/main/weeding_robot1.py)
> This code helps Thorvald detect weeds in front of it and spray over them.

 

