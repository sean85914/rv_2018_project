#!/usr/bin/env python
import rospy
import tf
import serial
global x, y, theta, v_L, v_R, v_x, v_y
x = 0
y = 0
theta = 0
v_L = 0
v_R = 0
v_x = 0
v_y = 0

br = tf.TransformBroadcaster()

def read_data(event):
	global str_
	str_ = str('')
	while ard.inWaiting():
		str_ = ard.readline()
	split_str = str_.split(' ')
	if len(split_str) != 8:
		global x, y, theta
		br.sendTransform((x, y, 0),
				 tf.transformations.quaternion_from_euler(0, 0, theta),
				 rospy.Time.now(),
				 'odom',
				 'map')
	else:
		try:
			global x, y, theta, v_L, v_R, v_x, v_y, omega
			x 	= float(split_str[0])
			y 	= float(split_str[1])
			theta   = float(split_str[2])
			v_L     = float(split_str[3])
			v_R     = float(split_str[4])
			v_x     = float(split_str[5])
			v_y     = float(split_str[6])
			omega   = float(split_str[7])
			br.sendTransform((x, y, 0),
				 	tf.transformations.quaternion_from_euler(0, 0, theta),
				 	rospy.Time.now(),
				 	'odom',
				 	'map')
			print("x: ", x,", y: " , y, ", theta: ", theta)
		except ValueError:
			pass

if __name__ == '__main__':
	rospy.init_node('whel_odom_node', anonymous = False)
	port = rospy.get_param("~port", "/dev/ttyACM0") # default port: /dev/ttyUSB0
	ard = serial.Serial(port, 9600)
	rospy.Timer(rospy.Duration.from_sec(0.1), read_data)
	rospy.spin()
