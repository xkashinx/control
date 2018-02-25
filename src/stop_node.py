#!/usr/bin/env python

import rospy
from control.msg import drive_param
from control.msg import pid_input

rospy.init_node('stop_node', anonymous=True)
pub = rospy.publisher('control/drive_parameters', drive_param, queue_size=10)

msg = drive_param()
msg.velocity = 0
msg.angle = 0
pub.publish(msg)
