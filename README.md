# Project : Pursuit-Evasion Robot Using ROS-Kobuki-Turtlebot
The project is done at University of Alberta (UoA) for the course CMPUT 412 : Experimental Robotics.

## Overview

The evasion robot tries to escape the pursuit robot and avoid any obstacle . The pursuit robot tries to catch the evasion robot at a safe deistance. The evasion robot has the wander or random walk property. On the other hand the pursuit robot has the follower proerty. Both of them use depth sensor for navigation.

## Dependencies

We tested our project on the following environment.
* Ubuntu 14.04
* Python 2.7.6
* ROS Indigo
* Numpy
* Matplotlib
* OpenCV-Python 2.4.8

How to configure onboard RasberryPi for WiFI:
```
ssh ubuntu@192.168.2.111


```
How to configure Joy:
http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick

## How to run 
### Evasion Robot
### Turtlebot Gazebo Simulation
```
roslaunch turtlebot_gazebo turtlebot_world.launch
cd catkin_ws
source devel/setup.bash
chmod +x fileName.py
catkin_make
rosrun packageName winder_sim.py
```
### Kobuki Turtlebot
```
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup 3dsensor.launch
roslaunch packageName standalone.launch #refer standalone.launch in project files
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
rosrun packageName wander.py cmd_vel:=cmd_vel/velocityramp #uses standalone.launch
```

### Pursuit Robot
### Turtlebot Gazebo Simulation
```
roslaunch turtlebot_gazebo turtlebot_world.launch
cd catkin_ws
source devel/setup.bash
chmod +x fileName.py
catkin_make
rosrun packageName follower_sim.py 
```
### Kobuki Turtlebot
```
roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_bringup 3dsensor.launch
roslaunch packageName standalone.launch #refer standalone.launch in project files
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
rosrun packageName follower.py cmd_vel:=cmd_vel/velocityramp #uses standalone.launch
```

## Project Procedure
The evasion robot is subscribed to the scan topics. It checks the minimum value of the vector excluding the NaN values.If it finds something within 0.8 m it turns for a fixed time and again moves forward.If forward moving exceeds some duration it turns again.This mechanism helps escaping from the other robot. See the following video below.

The pursuit robot tries to follow the nearest object. It sees object at certain distance , if no object found it drives at diagonally.Whenever an object is found it maintains a safe distance.The proportionate behavior with respect to the error is calculated in terms of tanh() function.We know tanh is a zero mean function and min value and max value is 1.So we multiplied the tanh() output with our maximum speed over x direction.For angular z we took the normalized position value with respect to half the vector(320) for depth resolution 640x480.


## Discussion

The pursuit robot can not differentiate between a static object and a moving boject. It always follows the distance nearest to it and hence often get stuck at the walls or starts follwing other moving oject nearest to it.Sometimes if the evasion robot is too close it sees it as blank as the sensor readings are NaN which are discarded.Velocity ramp discussed in [1] is used.

## Future Work
The follower behavior can be improved with RGB image processing.Combining RGB-D and pretrained model of the robot will help more prcise following.

## Authors

* [Nazmus Sakib](https://github.com/nsa31)
* **Vivian**
## Acknowledgement 

* [1] [Programming Robots with ROS](https://github.com/osrf/rosbook/blob/master)
* [2] [Maximum Security Bot](http://people.cornellcollege.edu/smikell15/MAX)
