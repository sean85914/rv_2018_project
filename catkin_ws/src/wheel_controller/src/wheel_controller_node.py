#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT

global width
width = 0.291

class WheelController(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		rospy.loginfo("[%s] Initializing ..." %(self.node_name))
		# Motors
		self.mh = Adafruit_MotorHAT(addr=0x60)
		self.left_motor = self.mh.getMotor(1)
		self.right_motor = self.mh.getMotor(2)
		# Motor state and pwm
		self.state_l = None
		self.state_r = None
		self.pwm_l = None
		self.pwm_r = None
		# Motor velocity
		self.v_L = None
		self.v_R = None
		# Emergency stop flag
		self.e_stop = None
		# Subscribers
		self.sub_car_cmd = rospy.Subscriber("car_cmd", Twist, self.cb_car_cmd ,queue_size=1)
		self.sub_e_stop  = rospy.Subscriber("e_stop", Bool, self.cb_e_stop, queue_size = 10)

	def cb_e_stop(self, msg):
		self.e_stop = msg.data

	def cb_car_cmd(self, twist_msg):
		if(self.e_stop == True):
			self.left_motor.setSpeed(0)
			self.right_motor.setSpeed(0)
			self.left_motor.run(4)
			self.right_motor.run(4)
	
			rospy.sleep(0.1)
		else:
			if(twist_msg.linear.x != 0):
				self.v_L = twist_msg.linear.x - width/2 * twist_msg.angular.z
				self.v_R = twist_msg.linear.x + width/2 * twist_msg.angular.z
				if(twist_msg.linear.x > 0):
					self.state_l = self.state_r = 1
				else:
					self.state_l = self.state_r = 2
				self.pwm_l = int(abs(self.v_L * 10000 / 7.48))
				self.pwm_r = int(abs(self.v_R * 10000 / 6.30))

			else:
				if(twist_msg.angular.z > 0):
					self.state_l = 4	
					self.state_r = 1
					self.v_R = width * twist_msg.angular.z
					self.pwm_l = 0
					self.pwm_r = int(abs(self.v_R * 10000 / 6.63))
				else:
					self.state_l = 1
					self.state_r = 4
					self.v_L = width * twist_msg.angular.z
					self.pwm_l = int(abs(self.v_L * 10000 / 7.48))
					self.pwm_r = 0
		
			if(self.pwm_l > 255):
				self.pwm_l = 255
			if(self.pwm_r > 255):
				self.pwm_r = 255
		
			self.left_motor.setSpeed(self.pwm_l)
			self.right_motor.setSpeed(self.pwm_r)
			
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
	rospy.init_node('wheel_controller_node', anonymous = False)
	wheel_controller = WheelController()
	rospy.on_shutdown(wheel_controller.on_shutdown)
	rospy.spin()

