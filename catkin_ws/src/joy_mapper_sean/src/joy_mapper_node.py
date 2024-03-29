#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class JoyMapper(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initilaizing ..."%(self.node_name))
		
		self.joy = None

		self.pub_car_cmd = rospy.Publisher("car_cmd", TwistStamped, queue_size=1)
		self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
		# Parameters
		self.v_gain = rospy.get_param("speed_gain", 0.2)
		self.omega_gain	= rospy.get_param("omega_gain", 0.4)

	def cbJoy(self, joy_msg):
		self.joy = joy_msg
		self.publishControl()

	def publishControl(self):
		car_cmd = TwistStamped()
		car_cmd.header.stamp = rospy.Time.now()
		car_cmd.twist.linear.x = self.joy.axes[1] * self.v_gain
		car_cmd.twist.angular.z = self.joy.axes[3] * self.omega_gain

		self.pub_car_cmd.publish(car_cmd)

if __name__ == "__main__":
	rospy.init_node("joy_mapper", anonymous = False)
	joy_mapper = JoyMapper()
	rospy.spin()
