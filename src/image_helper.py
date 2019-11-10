import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


def detect_red(image):
    # Isolate the blue colour in the image as a binary image
    mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
    # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    # Obtain the moments of the binary image
    M = cv2.moments(mask)
    # Calculate pixel coordinates for the centre of the blob
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])
 

# Detecting the centre of the green circle
def detect_green(image):
    mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])


  # Detecting the centre of the blue circle
def detect_blue(image):
    mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

  # Detecting the centre of the yellow circle
def detect_yellow(image):
    mask = cv2.inRange(image, (0, 200, 200), (0, 255, 255))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return np.array([cx, cy])

def isolate_orange(image):
    mask = cv2.inRange(image, (0, 100, 100), (100, 200, 250))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours

def match_template(image, template):
    sums = []
    contours = isolate_orange(image)
    mask = np.zeros(image.shape)
    mask = cv2.inRange(cv2.drawContours(mask, contours, -1, (255, 255, 255), 1), (200, 200, 200), (255, 255, 255))
    dist = cv2.distanceTransform(cv2.bitwise_not(mask), cv2.DIST_C, 0)

    for contour in contours:
      x, y, width, height = cv2.boundingRect(contour)
      if width <= 1 or height <= 1:
        sums.append(np.inf)
        continue
      shape = mask[y:y+height, x:x+width]
      shape = cv2.resize(shape, (template.shape[1], template.shape[0]), interpolation=cv2.INTER_AREA)
      cv2.imshow('i', shape * dist[y:y+shape.shape[0], x:x+shape.shape[1]])
      cv2.waitKey(0)

      sums.append(np.sum(shape * dist[y:y+shape.shape[0], x:x+shape.shape[1]]))
      
  
    return(contours[np.argmin(sums)])
      



