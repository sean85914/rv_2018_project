#!/usr/bin/env python
import rospy
import tf
import serial
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from robotx_msgs.msg import Two_wheels_velocity

global theta, v_L, v_R, v_x

theta = 0
v_L = 0
v_R = 0
v_x = 0

def read_data(event):
	pub_odom = rospy.Publisher("/odom", Odometry, queue_size = 20)
	pub_velocity = rospy.Publisher("/two_wheels_feedback", Two_wheels_velocity, queue_size = 20)
	global str_
	str_ = str('')
	seq = 0
	while ard.inWaiting():
		str_ = ard.readline().strip()
	split_str = str_.split(' ')
	if len(split_str) != 4:
		global x, y, theta, v_x
		odom = Odometry()
		odom.header.seq = seq
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.child_frame_id = "base_link"
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
		odom.pose.pose.orientation.x = odom_quat[0]
		odom.pose.pose.orientation.y = odom_quat[1]
		odom.pose.pose.orientation.z = odom_quat[2]
		odom.pose.pose.orientation.w = odom_quat[3]
		# Pose variances: TBD
		odom.pose.covariance[35] = 0.1 # Theta
		odom.twist.twist.linear.x = v_x
		# Twist variances: TBD
		odom.twist.covariance[0] = 0.5 # v_x
		seq = seq + 1
		pub_odom.publish(odom)
	else:
		try:
			theta   = float(split_str[0])
			v_L     = float(split_str[1])
			v_R     = float(split_str[2])
			v_x     = float(split_str[3])
			odom = Odometry()
			odom.header.seq = seq
			odom.header.stamp = rospy.Time.now()
			odom.header.frame_id = "odom"
			odom.child_frame_id = "base_link"
			odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
			odom.pose.pose.orientation.x = odom_quat[0]
			odom.pose.pose.orientation.y = odom_quat[1]
			odom.pose.pose.orientation.z = odom_quat[2]
			odom.pose.pose.orientation.w = odom_quat[3]
			odom.pose.covariance[35] = 0.1 # Theta
			odom.twist.twist.linear.x = v_x
			odom.twist.covariance[0] = 0.5
			pub_odom.publish(odom)
			rospy.loginfo("[%s] v_x: %s, theta: %s" %(rospy.get_name(), v_x, theta))
			seq = seq + 1
			feedback = Two_wheels_velocity()
			feedback.left = v_L
			feedback.right = v_R
			pub_velocity.publish(feedback)
		except ValueError:
			pass

if __name__ == '__main__':
	rospy.init_node('whel_odom_node', anonymous = False)
	port = rospy.get_param("~port", "/dev/ttyACM0") # default port: /dev/ttyUSB0
	ard = serial.Serial(port, 9600)
	rospy.loginfo("[%s] Flush first 20 data..." %(rospy.get_name()))
	for i in range(20):
		ard.readline()
	rospy.Timer(rospy.Duration.from_sec(0.01), read_data) # 100Hz
	rospy.spin()
