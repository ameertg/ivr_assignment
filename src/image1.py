#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

import image_helper


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a publisher to send joint coordinates from camera1 to joints
    self.joints_pub1 = rospy.Publisher("joints1", Float64MultiArray, queue_size = 10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize publisher to write orange ball coordinates to target1
    self.target_pub1 = rospy.Publisher("target1", Point, queue_size=1)


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    
    # Get joint coordinates
    red = image_helper.detect_red(self.cv_image1)
    blue = image_helper.detect_blue(self.cv_image1)
    green = image_helper.detect_green(self.cv_image1)
    yellow = image_helper.detect_yellow(self.cv_image1)

    # Write coords to Float64MultiArray
    self.coords = Float64MultiArray()
    self.coords.data = np.array([red, blue, green, yellow]).flatten()

    # Load template files
    self.ball_template = cv2.inRange(cv2.imread("ball1.png"), (200, 200, 200), (255, 255, 255))
    # Detect orange ball
    ball = image_helper.match_template(self.cv_image1, self.ball_template)
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


    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.joints_pub1.publish(self.coords)
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.target_pub1.publish(self.ball_coords)
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


