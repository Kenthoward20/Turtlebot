# Turtlebot

This package is made for **MLDA Robotics ROS Workshop 2**.  

>For **ROS Workshop 4 (Localization and Navigation with ROS)**, please switch to the `nav-workshop` branch (and `nav-workshop-ans` branch for sample answer).

This package uses a modified turtlebot3 gazebo simulation to control a turtlebot to hit a pillar. The package turtlebot3_controller contains a controller capable of detecting the nearest object based on subscribed LIDAR data and publishing velocity command to the robot.

## Preview
![Turtlebot_demo](img/turtlebot_demo.png)

## Usage
To use the turtlebot3_controller package, run the following commands:
```sh
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_controller turtlebot3_controller.launch
```
