# Introduction to Vision and Robotics Assignment

This was a coursework project where we were tasked in groups of two to control an open-chain robot arm to track an orange ball.
The project was broken into two parts, one involving computer vision and the second focusing on robot control and manipulation.
The robot was implemented in ROS Gazebo and all comunication was done via ROS publishing. <br> <br>
The project involved us using computer vision algorithms to detect correctly detect the target and ignore a secondary target of the same colour. We also used vision to detect the coloured robot joints and estimate the angles of rotation by solving the inverse kinematics.
The second part of the project involved using PD control to manipulate the robot end-effector to track the target.
