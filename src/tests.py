#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
import time

import control_helper

class tester:

    def __init__(self):
    	# initialize the node named image_processing
        rospy.init_node('tester', anonymous=True)

        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.targetX_pub = rospy.Publisher("/target/targetX", Float64, queue_size = 10)
        self.targetY_pub = rospy.Publisher("/target/targetY", Float64, queue_size = 10)
        self.targetZ_pub = rospy.Publisher("/target/targetZ", Float64, queue_size = 10)
        self.endX_pub = rospy.Publisher("/robot/endX", Float64, queue_size = 10)
        self.endY_pub = rospy.Publisher("/robot/endY", Float64, queue_size = 10)
        self.endZ_pub = rospy.Publisher("/robot/endZ", Float64, queue_size = 10)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.robot_q_sub = rospy.Subscriber("/robot/joint_states",JointState,self.robotQCallback)
        self.target_pos_sub = rospy.Subscriber("/target/joint_states",JointState,self.targetPosCallback)
        self.robot_pos_sub = rospy.Subscriber("/end_effector",Float64MultiArray,self.robotPosEstCallback)

        self.robotQ = None
        self.ImagePosEstimate = None
        self.target_position = np.array([0.0,0.0,0.0])

    def robotQCallback(self,data):
        self.robotQ = np.asarray(data.position)

    def robotPosEstCallback(self,data):
        self.ImagePosEstimate = data.data#np.asarray(data.position)

    def targetPosCallback(self,data):
        self.target_position = np.asarray(data.position)
        self.accuracyTest()

    def setQ(self,qs):
        self.joint1=Float64()
        self.joint1.data= qs[0]
        self.joint2=Float64()
        self.joint2.data= qs[1]
        self.joint3=Float64()
        self.joint3.data= qs[2]
        self.joint4=Float64()
        self.joint4.data= qs[3]
        # Publish the results
        try: 
            self.robot_joint1_pub.publish(self.joint1)
            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)


    def tenPosTest(self):
        jointSet = np.random.rand(10,4)*np.pi - (np.pi/2)
        print("10 random states test:")
        print("\t\tFK Estimate\t\t\t\tImage Estimate")
        for jointState in jointSet:
            self.setQ(jointState)
            time.sleep(5) # need to wait for ImagePosEstimate to update
            FKPosEstimate = control_helper.forward_kinematics(jointState)
            ImagePosEstimate = self.ImagePosEstimate

            print("{}  {}  {}\t{}  {}  {}".format(FKPosEstimate[0],FKPosEstimate[1],FKPosEstimate[2],ImagePosEstimate[0],ImagePosEstimate[1],ImagePosEstimate[2]))
            self.ImagePosEstimate = None

    def accuracyTest(self):
        if (self.robotQ is None): return
        fk = control_helper.forward_kinematics(self.robotQ)
        # Publish the results
        try: 
            self.targetX_pub.publish(self.target_position[0])
            self.targetY_pub.publish(self.target_position[1])
            self.targetZ_pub.publish(self.target_position[2])
            self.endX_pub.publish(fk[0])
            self.endY_pub.publish(fk[1])
            self.endZ_pub.publish(fk[2])

        except CvBridgeError as e:
            print(e)

 # call the class
def main(args):
  t = tester()
  #t.tenPosTest()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)