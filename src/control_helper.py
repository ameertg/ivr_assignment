#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


def forward_kinematics(angles):
	theta1 = angles[0]
	theta2 = angles[1]+(np.pi/2)
	theta3 = angles[2]
	theta4 = angles[3]

	c1,c2,c3,c4 = np.cos(theta1), np.cos(theta2), np.cos(theta3), np.cos(theta4)
	s1,s2,s3,s4 = np.sin(theta1), np.sin(theta2), np.sin(theta3), np.sin(theta4)

	x = -2*c1*c2*c3*c4 + 3*c1*c2*c3 - 2*s1*s3*c4 + 3*s1*s3 - 2*c1*s2*s4
	y = 2*s1*c2*c3*c4  + 3*s1*c2*c4 + 2*c1*s3*c4 + 3*c1*s3 - 2*s1*s2*s4
	z = 2*s2*c3*c4 + 3*c3 + 2*c2*s4 + 2


	return(np.array([x,y,z]))

def calculate_jacobian(angles):
	theta1 = angles[0]
	theta2 = angles[1]+(np.pi/2)
	theta3 = angles[2]
	theta4 = angles[3]

	c1,c2,c3,c4 = np.cos(theta1), np.cos(theta2), np.cos(theta3), np.cos(theta4)
	s1,s2,s3,s4 = np.sin(theta1), np.sin(theta2), np.sin(theta3), np.sin(theta4)

	jacobian = np.array([[
	2*s1*c2*c3*c4-3*s1*c2*c3-2*c1*s3*c4+3*c1*s3+2*s1*s2*s4,
	2*c1*s2*c3*c4-3*c1*s2*c3-2*c1*c3*s4,
	2*c1*c2*s3*c4-3*c1*c2*s3-2*s1*c3*c4+3*s1*c3,
	2*c1*c2*c3*s4+2*s1*s3*s4-2*c1*s2*c4],
	[2*s1*c2*c3*c4+3*c1*c2*c3-2*s1*s3*c4-3*s1*s3-2*c1*s2*s4,
	-2*s1*s2*c3*c4-3*s1*s2*c3-2*s1*c2*s4,
	-2*s1*c2*s3*c4-3*s1*c2*s3+2*c1*c3*c4+3*c1*c3,
	-2*s1*c2*c3*s4-2*c1*s3*s4-2*s1*s2*c4],
	[0,
	2*c2*c3*c4-2*s2*s4,
	-2*s2*s3*c4-3*s3,
	-2*s2*c3*s4+2*c2*c4
	]])

	return jacobian

print(forward_kinematics([(3*np.pi/2),0,0,(np.pi/2)]))