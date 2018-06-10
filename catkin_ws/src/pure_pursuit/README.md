# Pure Pursuit
## Purpose: 
### Given waypoint and try to follow the waypoint  
### ROS Topic
#### Subscribe: 
nav_msgs/Odometry: /odometry/filtered
#### Publish: 
geometry_msgs/Twist: /car_cmd


std_msg/Bool: /pure_pursuit/finished


geometry_msgs/Point: /pure_pursuit/lookahead

# How to run

```
	$ roslaunch pure_pursuit pure_pursuit.launch lookahead:=[XXX]
	(default lookahead value is 0.5m)
```
