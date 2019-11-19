#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
import math
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, Float64
from scipy.optimize import least_squares
from math import sin, cos


import image_helper

# This class computes the 3rd dimension for all 
# pairs of 2d-coords published by the image processing nodes.
class compute_dimensions:

  def __init__(self):
    rospy.init_node('compute_dimensions', anonymous=True)
    self.joints_pub = rospy.Publisher("/joints", Float64MultiArray, queue_size=10)
    self.target_pub = rospy.Publisher("/target1", Float64MultiArray, queue_size=10)
    # Wait for two messages to appear one from joints1 and one from joints2
    self.joints_sub1 = message_filters.Subscriber("/joints1",Float64MultiArray)
    self.joints_sub2 = message_filters.Subscriber("/joints2",Float64MultiArray)
    self.TS = message_filters.ApproximateTimeSynchronizer([self.joints_sub1, self.joints_sub2], 1, 0.1, allow_headerless = True)

  # Callback function for joints
  def knit_joints(self,data1, data2):
    self.joint_coords = [0 for i in range(5)]
    # Separate message data into individual joints in 2d
    joints1 = zip(*[iter(data1.data)]*2)
    x1 = np.array([x for x, z in joints1])
    joints2 = zip(*[iter(data2.data)]*2)
    y1 = np.array([y for y, z in joints2])

    # Check for occlusion
    for i in range(len(joints1)):
      if math.isnan(joints1[i][0]):
        y, z = joints2[i]
        ys = y1
        ys[np.isnan(x1)] = np.nan
        closest_y = np.nanargmin(ys - y)
        x = joints2[closest_y][0]
        self.joint_coords[i] = (x, y, z)

      elif math.isnan(joints2[i][0]):
        x, z = joints1[i]
        xs = x1
        xs[np.isnan(y1)] = np.nan
        closest_x = np.nanargmin(xs - x)
        y = joints2[closest_x][0]
        self.joint_coords[i] = (x, y, z)

      else:
        x, z1 = joints1[i]
        y, z2 = joints2[i]
        self.joint_coords[i] = (x, y, (z1+z2)/2)

    

    # Scale and center
    self.joint_coords = np.array(self.joint_coords) - np.array(self.joint_coords[0])
    self.joint_coords = self.joint_coords * (2 / self.joint_coords[2][2])
    
    
    def f(x):
      a, b, c, d = tuple(x)
      f1, f2, f3 = 2*sin(a)*sin(b)*cos(c)*cos(d) + 3*sin(a)*sin(b)*cos(c) + 2*sin(a)*sin(d)*cos(b) + 2*sin(c)*cos(a)*cos(d) + 3*sin(c)*cos(a), 2*sin(a)*sin(c)*cos(d) + 3*sin(a)*sin(c) - 2*sin(b)*cos(a)*cos(c)*cos(d) - 3*sin(b)*cos(a)*cos(c) - 2*sin(d)*cos(a)*cos(b), -2*sin(b)*sin(d) + 2*cos(b)*cos(c)*cos(d) + 3*cos(b)*cos(c) + 2
      f4 = 3*cos(b)*cos(c) + 2
      return np.array([f1 - self.joint_coords[3][0], f2 - self.joint_coords[3][1], f1 - self.joint_coords[3][2], f1 - self.joint_coords[2][2]])

    
    self.joint_states = Float64MultiArray()
    self.joint_states.data = np.array(least_squares(f, np.zeros(4), bounds=(-0.5*np.pi, 0.5*np.pi)).x)
    self.target_coords = Float64MultiArray()
    self.target_coords.data= self.joint_coords[4]

    self.joints_pub.publish(self.joint_states)
    self.target_pub.publish(self.target_coords)


# call the class
def main(args):
  comp_dim = compute_dimensions()
  comp_dim.TS.registerCallback(comp_dim.knit_joints)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
