#!/usr/bin/env python

import rospy

from wheel_controller.srv import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def handle_e_stop(req):
	resp = e_stopResponse()
	if req.stop == True:
		resp.result = True
		stop = Bool()
		stop.data = True
		pub_e_stop.publish(stop)
	else:
		resp.result = False
	return resp

if __name__ == "__main__":
	rospy.init_node('set_goal_server_node')
	server = rospy.Service('e_stop', e_stop, handle_e_stop)
	pub_e_stop = rospy.Publisher('/e_stop', Bool, queue_size = 10)
	rospy.spin()