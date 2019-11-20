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

import control_helper

class controller:

  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('controller', anonymous=True)

    # initialize a publisher to send joints' angular position to the robot
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    #self.robot_pos_sub = rospy.Subscriber("/robot/joint_states",JointState,self.callback)
    self.robot_pos_sub = rospy.Subscriber("/joints",Float64MultiArray,self.callback)
    self.target_pos_sub = rospy.Subscriber("/target1",Float64MultiArray,self.targetPosCallback)    
    #self.target_pos_sub = rospy.Subscriber("/target/joint_states",JointState,self.targetPosCallback)
    

    # record the begining time
    self.time_trajectory = rospy.get_time()
    # initialize errors
    self.time_previous_step = np.array([rospy.get_time()], dtype='float64')     
    self.time_previous_step2 = np.array([rospy.get_time()], dtype='float64')   

    self.error = np.array([0.0,0.0,0.0], dtype='float64')  
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64') 

    self.end_effector_position = np.array([0.0,0.0,0.0,0.0])
    self.target_position = np.array([0.0,0.0,0.0])

#  def trajectory(self):
#    # get current time
#    cur_time = np.array([rospy.get_time() - self.time_trajectory])
#    x_d = float(6* np.cos(cur_time * np.pi/100))
#    y_d = float(6 + np.absolute(1.5* np.sin(cur_time * np.pi/100)))
#    return np.array([x_d, y_d])

  def control_closed(self,position):
    # P gain
    K_p = np.array([[10,0,0],[0,10,0],[0,0,10]])
    # D gain
    K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
    # estimate time step
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time

    # robot end-effector position (xyz)
    pos = control_helper.forward_kinematics(position)
    # desired trajectory
    pos_d= self.target_position
    # estimate derivative of error
    self.error_d = ((pos_d - pos) - self.error)/dt
    # estimate error
    self.error = pos_d-pos

    #end effector position (angles)
    q = position
    print("end effector: {}".format(pos))
    print("target: {}".format(self.target_position))
    J_inv = np.linalg.pinv(control_helper.calculate_jacobian(q))  # calculating the psudeo inverse of Jacobian
    dq_d =np.dot(J_inv, ( np.dot(K_d,self.error_d.transpose()) + np.dot(K_p,self.error.transpose()) ) )  # control input (angular velocity of joints)
    q_d = q + (dt * dq_d)  # control input (angular position of joints)
    return q_d

  def robotPosCallback(self,data):
    thing

  def targetPosCallback(self,data):
    self.target_position = np.asarray(data.data)
    #self.target_position = np.asarray(data.position)

  # Recieve data, process it, and publish
  def callback(self,data):

    
    # compare the estimated position of robot end-effector calculated from images with forward kinematics(lab 3)
    #x_e = self.forward_kinematics(cv_image)
    #x_e_image = self.detect_end_effector(cv_image)
    #self.end_effector=Float64MultiArray()
    #self.end_effector.data= x_e_image	

    # send control commands to joints (lab 3)
    q_d = self.control_closed(data.data)
    #q_d = self.control_closed(data.position)
    self.joint1=Float64()
    self.joint1.data= q_d[0]
    self.joint2=Float64()
    self.joint2.data= q_d[1]
    self.joint3=Float64()
    self.joint3.data= q_d[2]
    self.joint4=Float64()
    self.joint4.data= q_d[3]

    # Publish the results
    try: 
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  c = controller()
  #c.callback()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
