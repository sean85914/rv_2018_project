from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import serial

ard = serial.Serial('/dev/ttyACM0', 9600)
f = open("const_pwm_150.txt", 'w')
mh = Adafruit_MotorHAT(addr = 0x60)
left = mh.getMotor(1)
right = mh.getMotor(2)

left.setSpeed(150)
right.setSpeed(150)
try:
	while(1):
		left.run(1)
		right.run(1)
		f.write(ard.readline())
except KeyboardInterrupt:
	left.run(4)
	right.run(4)
	f.close()
