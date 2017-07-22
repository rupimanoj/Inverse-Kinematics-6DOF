#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
import numpy as np

q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3,alpha4, alpha5,alpha6 = symbols('alpha0:7')

s ={alpha0 : 0      ,a0:0       ,d1: 0.75   ,
	alpha1 : -pi/2  ,a1: 0.35   ,d2: 0      , q2: -pi/2 + q2,
	alpha2 : 0      ,a2: 1.25   ,d3: 0      ,
	alpha3 : -pi/2  ,a3: -0.054 ,d4: 1.5    ,
	alpha4 : pi/2   ,a4:0       ,d5: 0      ,
	alpha5 : -pi/2  ,a5:0       ,d6: 0      ,
	alpha6 : 0      ,a6:0       ,d7: 0.303  , q7:0}

R_z = Matrix([[cos(np.pi),-sin(np.pi),0],[sin(np.pi),cos(np.pi),0],[0,0,1]])
R_y = Matrix([[cos(-np.pi/2),0,sin(-np.pi/2)],[0,1,0],[-sin(-np.pi/2),0,cos(-np.pi/2)]])
R_x = Matrix([[1,0,0],[0,cos(np.pi/2),-sin(np.pi/2)],[0,sin(np.pi/2),cos(np.pi/2)]])
R_corr = (R_z * R_y)  #intrinsic rotation in the order of Z and Y axis

R_zr = Matrix([[cos(-np.pi),-sin(-np.pi),0],[sin(-np.pi),cos(-np.pi),0],[0,0,1]])
R_yr = Matrix([[cos(np.pi/2),0,sin(np.pi/2)],[0,1,0],[-sin(np.pi/2),0,cos(np.pi/2)]])
r_corr_reverse = R_yr * R_zr

def get_homogeneous_matrix(a,alpha,d,q):

	T = Matrix([[           cos(q),             -sin(q),            0,                   a],
				[sin(q)*cos(alpha),     cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)* d],
				[sin(q)*sin(alpha),     cos(q)*sin(alpha),  cos(alpha),     cos(alpha) * d],
				[           0   ,                   0   ,           0,          1         ]])

	return T

def get_wrist_center(x,y,z,roll,pitch,yaw): #gamma - roll beta -yaw alpha - pitch
    #a1 = cos(a) * cos(b)
    #d1 = sin(a)*cos(b)*1.0
    #g1 = -sin(b)
    R0_G = tf.transformations.euler_matrix(roll,pitch,yaw, axes='sxyz')
    a1,d1,g1 =  R0_G[0][0],R0_G[1][0],R0_G[2][0]
    d7_value = s[d7]
    WC_x =   x - (d7_value * a1 * 1.0) #in the world coordinates it is changing along x-direction
    WC_y =   y - (d7_value * d1 * 1.0)  
    WC_z =   z - (d7_value * g1 * 1.0) 
    return WC_x,WC_y,WC_z

def calculate_position_thetas(WC_x,WC_y,WC_z,alpha,beta,gamma):

	theta1 = atan2(WC_y,WC_x)
	X_new = 0.35 * cos(theta1)
	Y_new = 0.35 * sin(theta1)
	x2,y2,z2 = X_new,Y_new,s[d1]

	L2_3 = s[a2]
	L2_WC = sqrt((WC_x - x2) ** 2 + (WC_y - y2) ** 2 + (WC_z - z2) ** 2)
	L3_WC = sqrt(s[a3] ** 2 + s[d4] ** 2)

	theta3_existing = np.pi/2 - atan2(s[a3],s[d4])
	d3 = (L2_WC ** 2 - L3_WC ** 2 - L2_3 ** 2) / (2*L3_WC*L2_3)
	theta3_new = atan2(sqrt(1-d3**2),d3)
	theta3 = theta3_new - theta3_existing

	xy_proj = sqrt((WC_y-y2) ** 2 +  (WC_x - x2) ** 2)
	theta2_plane = atan2(WC_z- z2 ,xy_proj)
	d2 = ( L2_3 ** 2 + L2_WC ** 2 - L3_WC ** 2) / (2*L2_WC*L2_3)
	theta2_link = atan2(sqrt(1-d2**2),d2)
	theta2_vertical = theta2_plane + theta2_link
	theta2 = (np.pi/2) - (theta2_plane + theta2_link)
	return theta1,theta2,theta3

def calculate_orientation_thetas(theta1,theta2,theta3,roll,pitch,yaw,T0_3):	
	R0_3 = T0_3[0:3,0:3]
	R0_3_dh_value = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
	T0_6_world_value = tf.transformations.euler_matrix(roll, pitch, yaw, axes='sxyz')
	R0_6_world_value = T0_6_world_value[0:3,0:3]
	R0_6_dh_value = R0_6_world_value * r_corr_reverse
	R3_6_dh_value = Transpose(R0_3_dh_value) * R0_6_dh_value
	R3_6_tf_adjusted = R3_6_dh_value * R_x
	theta4,theta5,theta6 = tf.transformations.euler_from_matrix(np.array(R3_6_tf_adjusted).astype(np.float64), "ryzy")
	return theta4,theta5,theta6

def handle_calculate_IK(req):
	rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
	if len(req.poses) < 1:
		print "No valid poses received"
		return -1
	else:
		# Initialize service response
		joint_trajectory_list = []
		for x in xrange(0, len(req.poses)):
			# IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

			# Define DH param symbols

			T0_1 = get_homogeneous_matrix(a0,alpha0,d1,q1)
			T1_2 = get_homogeneous_matrix(a1,alpha1,d2,q2)
			T2_3 = get_homogeneous_matrix(a2,alpha2,d3,q3)
			T3_4 = get_homogeneous_matrix(a3,alpha3,d4,q4)
			T4_5 = get_homogeneous_matrix(a4,alpha4,d5,q5)
			T5_6 = get_homogeneous_matrix(a5,alpha5,d6,q6)

			T0_1 = T0_1.subs(s)
			T1_2 = T1_2.subs(s)
			T2_3 = T2_3.subs(s)
			T3_4 = T3_4.subs(s)
			T4_5 = T4_5.subs(s)
			T5_6 = T5_6.subs(s)

			T0_3 = T0_1 * T1_2 * T2_3




			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
				[req.poses[x].orientation.x, req.poses[x].orientation.y,
					req.poses[x].orientation.z, req.poses[x].orientation.w])
	 
			# Calculate joint angles using Geometric IK method
			alpha,beta,gamma = yaw,pitch,roll
			WC_x,WC_y,WC_z 	= get_wrist_center(px,py,pz,roll,pitch,yaw)
			theta1, theta2, theta3 = calculate_position_thetas(WC_x,WC_y,WC_z,alpha,beta,gamma)
			theta4, theta5, theta6 = calculate_orientation_thetas(theta1, theta2, theta3,roll,pitch,yaw,T0_3)

			# Populate response for the IK request
			# In the next line replace theta1,theta2...,theta6 by your joint angle variables
		joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
		joint_trajectory_list.append(joint_trajectory_point)

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)

def IK_server():
	# initialize node and declare calculate_ik service
	rospy.init_node('IK_server')
	s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
	print "Ready to receive an IK request"
	rospy.spin()


if __name__ == "__main__":
	IK_server()
