#!/usr/bin/env python

import rospy
import utm
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion, Vector3
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from gps_receiver.srv import *

global x_, y_
x_ = 0
y_ = 0

global goal_x, goal_y
goal_x = 0
goal_y = 0

global status
status = False

global seq
seq = 0

def handle_set_goal(req):
	utm_x, utm_y, _, _ = utm.from_latlon(req.lat, req.lon)
	
	resp = set_goalResponse()
	if(not status):
		global x_, y_
		print x_, y_
		resp.x = utm_x - x_
		resp.y = utm_y - y_
		global goal_x, goal_y
		goal_x = resp.x
		goal_y = resp.y
		rospy.loginfo("x: %s, y: %s" %(resp.x, resp.y))
	return resp

def cb_tf(msg):
	global x_, y_
	x_ = msg.transforms[0].transform.translation.x
	y_ = msg.transforms[0].transform.translation.y
	print x_, y_
	status = True
def pub_data(event):
	if(goal_x == 0 and goal_y == 0):
		return
	else:
		global seq
		global pub_marker
		marker = Marker()
		marker.header.seq = seq
		marker.header.stamp = rospy.Time.now()
		marker.header.frame_id = '/odom'
		marker.type = Marker.POINTS
		marker.action = Marker.ADD
		marker.pose.position.x = goal_x
		marker.pose.position.y = goal_y
		p = Point()
		p.x = goal_x
		p.y = goal_y
		marker.points.append(p)
		marker.scale = Vector3(0.05, 0.05, 0.05)
		marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.8)

		pub_marker.publish(marker)
		seq += 1

def set_goal_server():
	rospy.init_node('set_goal_server_node')
	server = rospy.Service('set_goal', set_goal, handle_set_goal)
	sub_tf = rospy.Subscriber("/tf_static", TFMessage, cb_tf, queue_size = 1)
	global pub_marker
	pub_marker = rospy.Publisher("/goal_marker", Marker, queue_size = 20)
	rospy.Timer(rospy.Duration(1), pub_data)
	rospy.spin()

if __name__ == "__main__":
	set_goal_server()
