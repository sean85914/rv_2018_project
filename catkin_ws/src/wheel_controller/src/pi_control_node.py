#!/usr/bin/env python

import rospy
import message_filters
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from Adafruit_MotorHAT import Adafruit_MotorHAT
from robotx_msgs.msg import Two_wheels_velocity

global WIDTH
WIDTH = 0.291

class WheelController(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		# Motors
		self.mh = Adafruit_MotorHAT(addr=0x60)
		self.left_motor = self.mh.getMotor(1)
		self.right_motor = self.mh.getMotor(2)
		# Motor state and pwm
		# state = 1: forward
		# state = 2: backward
		# state = 4: stop
		# pwm = 0~255
		self.state_l = None
		self.state_r = None
		self.pwm_l = None
		self.pwm_r = None
		# Motor velocity
		self.v_L = None
		self.v_R = None
		# Gain
		self.kp_R = 100
		self.kp_L = 100
		self.ki_R = 20
		self.ki_L = 20
		# Error sum
		self.error_sum_R = 0
		self.error_sum_L = 0
		# Emergency stop flag
		self.e_stop = None
		# Subscribers
		sub_car_cmd = message_filters.Subscriber("/car_cmd", TwistStamped)
		sub_feedback = message_filters.Subscriber("/two_wheels_feedback", Two_wheels_velocity)
		ts = message_filters.ApproximateTimeSynchronizer([sub_car_cmd, sub_feedback], slop = 0.5, queue_size = 20)
		ts.registerCallback(self.cb_control)	
		self.sub_e_stop  = rospy.Subscriber("e_stop", Bool, self.cb_e_stop, queue_size = 20)

	def cb_e_stop(self, msg):
		self.e_stop = msg.data

	def cb_control(self, car_cmd, feedback):
		print "hello"
		if(self.e_stop == True):
			self.left_motor.setSpeed(0)
			self.right_motor.setSpeed(0)
			self.left_motor.run(4)
			self.right_motor.run(4)
			rospy.loginfo("[%s] Emergency stop!" %(self.node_name))
			rospy.sleep(0.1)
		else:
			desired_v_R = car_cmd.twist.linear.x + WIDTH /2 * car_cmd.twist.angular.z
			desired_v_L = car_cmd.twist.linear.x - WIDTH /2 * car_cmd.twist.angular.z
			error_R = desired_v_R - feedback.right
			error_L = desired_v_L - feedback.left
			self.error_sum_R += error_R
			self.error_sum_L += error_L
			pwm_R = int(error_R * self.kp_R + self.error_sum_R * self.ki_R)
			pwm_L = int(error_L * self.kp_L + self.error_sum_L * self.ki_L)
			if pwm_R > 0:
				self.state_r = 1
				if pwm_R >= 255:
					pwm_R = 255
			if pwm_R < 0:
				self.state_r = 2
				pwm_R = -pwm_R
				if pwm_R >= 255:
					pwm_R = 255
			if pwm_L > 0:
				self.state_l = 1
				if pwm_L >= 255:
					pwm_L = 255
			if pwm_L < 0:
				self.state_l = 2
				pwm_L = -pwm_L
				if pwm_L >= 255:
					pwm_L = 255
			
			self.left_motor.setSpeed(pwm_L)
			self.right_motor.setSpeed(pwm_R)
			self.left_motor.run(self.state_l)
			self.right_motor.run(self.state_r)
			
			rospy.sleep(0.1)
	
	def on_shutdown(self):
		rospy.loginfo("[%s] Shutting down ..." %(self.node_name))
		self.left_motor.setSpeed(0)
		self.right_motor.setSpeed(0)
		self.left_motor.run(4)
		self.right_motor.run(4)
		rospy.sleep(1)
		del self.mh

if __name__ == "__main__":
	rospy.init_node('pid_controll_node', anonymous = False)
	wheel_controller = WheelController()
	rospy.on_shutdown(wheel_controller.on_shutdown)
	rospy.spin()

