#!/usr/bin/env python

import rospy
from control.msg import drive_param
from control.msg import pid_input
from std_msgs.msg import Bool

class Pid(object):
	def __init__(self):
		# Params
		self.kp = 3
		self.kd = 0.05
		self.prev_error = 0.0
		self.go = True

		# Publisher
		self.pub = rospy.Publisher('control/drive_parameters', drive_param, queue_size=1)

		# Subscribers
		rospy.Subscriber('control/error', pid_input, self.control)
		rospy.Subscriber('control/go', Bool, self.go_callback)

		rospy.loginfo("Started pic_controller\nListening to /control/error and /control/go")
		rospy.spin()

	def control(self, data):
		## Your code goes here
		# 1. Scale the error
		# 2. Apply the PID equation on error
		# 3. Make sure the error is within bounds





		msg = drive_param()
		msg.velocity = data.pid_vel
		error = data.pid_error
		angle_p = error * self.kp
		angle_d = self.kd * (error-self.prev_error)
		angle = angle_p+angle_d
		self.prev_error = error
		msg.angle = angle
		if not self.go:
			msg.velocity = 0.0
		self.pub.publish(msg)
		rospy.loginfo("\nvel: %.0lf\ngo: %i", msg.velocity, self.go)

	def go_callback(self, msg):
		self.go = msg.data
		rospy.loginfo("\nEntered go_callback\ngo: %i", self.go)

if __name__ == '__main__':
	rospy.init_node('pid_controller', anonymous=True)
	my_node = Pid()
