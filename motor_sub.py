#!/usr/bin/env python
import rclpy
from rclpy.node import Node

import time
import threading
from geometry_msgs.msg import Twist
from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

class DaguWheelsDriver:
	LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor  
	LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor  
	RIGHT_MOTOR_MIN_PWM = 60       # Minimum speed for right motor  
	RIGHT_MOTOR_MAX_PWM = 255      # Maximum speed for right motor  
	SPEED_TOLERANCE = 1.e-2       # speed tolerance level

	def __init__(self, verbose=False, debug=False, left_flip=False, right_flip=False):
		self.motorhat = Adafruit_MotorHAT(addr=0x60)
		self.leftMotor = self.motorhat.getMotor(1)
		self.rightMotor = self.motorhat.getMotor(2)
		self.verbose = verbose or debug
		self.debug = debug

		self._lock=threading.RLock()
		self._streaming=False
		self._ep=0
		# Set directions based on flip property
		self.left_sgn = 1.0
		if left_flip:
			self.left_sgn = -1.0
		self.right_sgn = 1.0
		if right_flip:
			self.right_sgn = -1.0
		# Set initial speeds to zero
		self.leftSpeed = 0.0
		self.rightSpeed = 0.0
		self.updatePWM()

	def PWMvalue(self, v, minPWM, maxPWM):
		pwm = 0
		if fabs(v) > self.SPEED_TOLERANCE:
			pwm = int(floor(fabs(v) * (maxPWM - minPWM) + minPWM))
		return min(pwm, maxPWM)

	def updatePWM(self):
		vl = self.leftSpeed*self.left_sgn
		vr = self.rightSpeed*self.right_sgn

		pwml = self.PWMvalue(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
		pwmr = self.PWMvalue(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)

		if fabs(vl) < self.SPEED_TOLERANCE:
			leftMotorMode = Adafruit_MotorHAT.RELEASE
		elif vl > 0:
			leftMotorMode = Adafruit_MotorHAT.FORWARD
		elif vl < 0: 
			leftMotorMode = Adafruit_MotorHAT.BACKWARD

		if fabs(vr) < self.SPEED_TOLERANCE:
			rightMotorMode = Adafruit_MotorHAT.RELEASE
			pwmr = 0
		elif vr > 0:
			rightMotorMode = Adafruit_MotorHAT.FORWARD
		elif vr < 0: 
			rightMotorMode = Adafruit_MotorHAT.BACKWARD

		self.leftMotor.setSpeed(pwml)
		self.leftMotor.run(leftMotorMode)
		self.rightMotor.setSpeed(pwmr)
		self.rightMotor.run(rightMotorMode)

	def setWheelsSpeed(self, left, right):
		with self._lock:
			self.leftSpeed = left
			self.rightSpeed = right
			self.updatePWM()

	# Stop the streaming thread
	def StopStreaming(self):
		if (not self._streaming):
			raise Exception("Not streaming")
		self._streaming=False

	def Shutdown(self):
		with self._lock:
			self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
			self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
			del self.motorhat

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(Twist, 'motor_command', self.chatter_callback, 10)
        self.obj=DaguWheelsDriver()
    def chatter_callback(self, data):
        self.obj.setWheelsSpeed(data.linear.x,data.linear.y)



def callback(data):
	rospy.loginfo(data)
	obj=DaguWheelsDriver()
	obj.setWheelsSpeed(data.linear.x,data.linear.y)


def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
