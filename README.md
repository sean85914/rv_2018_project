# rv_2018_project
## Purpose: 
### Use self-designed AGV (as following image) to simulate RobotX competition task, mainly avoid obstacle one  
![](https://github.com/seanNCTU/rv_2018_project/blob/master/img/AGV.jpg)

## MCU:
### Arduino UNO
### Raspberry3, model B
### Notebook

## System framework: 
### Ubuntu 16.04 
### ROS, Kinetic

## Sensors:
### 12-volt motors w/ encoder (differential drive)
### Sparkfun razor 9-dof IMU
### G-STAR IV GPS receiver, type BU-353S4
### Velodyne 16-beam Lidar

# How to install

...
	$ source /opt/ros/kinetic/setup.bash
	$ cd
	$ git clone https://github.com/seanNCTU/rv_2018_project.git
	$ cd rv_2018_project/catkin_ws
	$ catkin_make install
	$ catkin_make
...

And upload wheel_odom/wheel_odom.ino to your Arduino
