# Project : Line-Lane-Follower-Robot Using ROS-Kobuki-Turtlebot
The project is done at University of Alberta (UoA) for the course CMPUT 412 : Experimental Robotics.

## Overview



## Dependencies

We tested our project on the following environment.
* Ubuntu 14.04
* Python 2.7.6
* ROS Indigo
* Numpy
* Matplotlib
* OpenCV-Python 2.4.8

How to configure Gazebo world for line follower:
clone followbot folder into your_workspace/src. For test 
```
cd your_workspace/src
roslaunch followbot course.launch
```
In our case, saving followbot files in a src/package didn't work. 

How to configure Joy:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

## How to run 
### Collect Camera Images
Run the following to record images into the imageFolder of your current directory.
```
mkdir imageFolder
cd imageFolder
rosbag record camera/rgb/image_raw --limit=60 -o bagFileName 
```
limit = 60 is the total number of frames collected.
Initialize joystick or remote keyboard teleop before to maneuver the robot and record.

Run the following to extract raw ROS bag files into JPEG.
```
rosrun packageName rosbag_play.py bagFileName.bag /Folderlocation/ camera/rgb/image_raw
```

### Perspective Calibration

### Line/Lane (yellow or white) Following Robot
#### In Turtlebot Gazebo Simulation 
```
roslaunch followbot course.launch
cd catkin_ws
source devel/setup.bash
chmod +x fileName.py
catkin_make
rosrun packageName white_yellow_line_follower_sim.py
#Or run this for lane following
rosrun packageName white_yellow_lane_follower_sim.py 
```
#### In Kobuki Turtlebot
```
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup 3dsensor.launch
roslaunch turtlebot_teleop logitech.launch 
```

If joystick appears on js1 other than js0:
```
ls -l /dev/input/js1
roslaunch packageName joy.launch #refer joy.launch in project files
```
```
cd catkin_ws
source devel/setup.bash
chmod +x fileName.py
catkin_make
rosrun packageName white_yellow_line_follower_sim.py
#Or run this for lane following
rosrun packageName white_yellow_lane_follower_sim.py 
```

## Project Description



## Discussion



## Future Work


## Authors

* [Nazmus Sakib](https://github.com/nsa31)
* **Vivian**
## Acknowledgement 

* [1] [Programming Robots with ROS](https://github.com/osrf/rosbook/blob/master)
* [2] [Maximum Security Bot](http://people.cornellcollege.edu/smikell15/MAX)
