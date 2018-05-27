#!/usr/bin/env python
import rospy
from robotx_msgs.msg import ObstaclePoseList, ObstaclePose

if __name__ == "__main__":
	rospy.init_node('test_node', anonymous=False)
	obstaclePoseList = ObstaclePoseList()
	obstaclePose = ObstaclePose()
	obstaclePose.x = 1
	obstaclePose.y = 2
	obstaclePose.z = 0
	obstaclePose.r = 1
	obstaclePoseList.list.append(obstaclePose)
	while 1:
		rospy.Publisher("/ObstacleList", ObstaclePoseList, queue_size = 1).publish(obstaclePoseList)	
