#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, Float64


import image_helper


class compute_dimensions:

  # Defines publisher and subscriber
  def __init__(self):
    rospy.init_node('compute_dimensions', anonymous=True)
    self.joints_pub = rospy.Publisher("/joints", Float64MultiArray, queue_size=1)
    self.joints_sub1 = message_filters.Subscriber("/joints1",Float64MultiArray)
    self.joints_sub2 = message_filters.Subscriber("/joints2",Float64MultiArray)
    self.TS = message_filters.ApproximateTimeSynchronizer([self.joints_sub1, self.joints_sub2], 10, 0.1, allow_headerless = True)

  def knit_joints(self,data1, data2):
    self.joints_3d = Float64MultiArray()
    red1, blue1, green1, yellow1 = zip(*[iter(data1.data)]*2)
    red2, blue2, green2, yellow2= zip(*[iter(data2.data)]*2)

    if np.nan in green1:
      if np.isclose(green2[1], yellow2[1]):
        green = [yellow1[0], green2[0], green2[1]]


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
