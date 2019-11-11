#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

import image_helper

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a publisher to send joint coordinates from camera1 to joints1
    self.joints_pub2 = rospy.Publisher("joints2", Float64MultiArray, queue_size = 10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize publisher to write orange ball coordinates to target2
    self.target_pub2 = rospy.Publisher("target2", Point, queue_size=1)

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    red = image_helper.detect_red(self.cv_image2)
    blue = image_helper.detect_blue(self.cv_image2)
    green = image_helper.detect_green(self.cv_image2)
    yellow = image_helper.detect_yellow(self.cv_image2)

    self.coords = Float64MultiArray()
    self.coords.data = np.array([red, blue, green, yellow]).flatten()

    # Load template files
    self.ball_template = cv2.inRange(cv2.imread("ball2.png"), (200, 200, 200), (255, 255, 255))
    # Detect orange ball
    ball = image_helper.match_template(self.cv_image2, self.ball_template)
    # Compute the coordinates of the centre of the ball
    self.ball_coords = Point()

    if ball is None:
        self.ball_coords.x = np.nan
        self.ball_coords.y = np.nan
        self.ball_coords.z = np.nan
    else:
        M = cv2.moments(ball)
        self.ball_coords.x = int(M['m10']/M['m00'])
        self.ball_coords.y = np.nan
        self.ball_coords.z = int(M['m01']/M['m00'])


    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.joints_pub2.publish(self.coords)
      self.target_pub2.publish(self.ball_coords)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


