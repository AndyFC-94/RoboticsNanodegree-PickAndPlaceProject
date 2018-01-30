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

def TF_Matrix(alpha, a, d, q):

	TF = Matrix([[                cos(q),            -sin(q),             0,              a],
		     [     sin(q)*cos(alpha),  cos(q)*cos(alpha),   -sin(alpha),  -sin(alpha)*d],
		     [     sin(q)*sin(alpha),  cos(q)*sin(alpha),    cos(alpha),   cos(alpha)*d],
		     [                     0,                  0,             0,             1]])
	return TF

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset
   	a0, a1, a2, a3, a4, a5, a6 = symbols ('a0:7') #link lenght
    	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    	#Joint angle symbols
    	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

	   
	# Create Modified DH parameters
	DH_Table = 	{ alpha0:  0,      a0:  0,      d1:  0.75, q1:    	q1, 
			 alpha1:  -pi/2., a1:  0.35,   d2:     0, q2:  -pi/2. + q2,
			 alpha2:  0,      a2:  1.25,   d3:     0, q3:          q3,
			 alpha3:  -pi/2., a3:  -0.054, d4:   1.5, q4:          q4,
			 alpha4:   pi/2.,  a4:  0,      d5:     0, q5:          q5,
			 alpha5:  -pi/2., a5:  0,      d6:     0, q6:          q6,
			 alpha6:  0,      a6:  0,      d7: 0.303, q7:          0 } 	
 

	# Create individual transformation matrices
	T0_1 =  TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
    	T1_2 =  TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
    	T2_3 =  TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
    	T3_4 =  TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
   	T4_5 =  TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
    	T5_6 =  TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
    	T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

    	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
	
	# Extract rotation matrices from the transformation matrices
	R0_1 = T0_1[0:3,0:3]
	R1_2 = T1_2[0:3,0:3]
	R2_3 = T2_3[0:3,0:3]
        ###


        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo

	    r, p, y = symbols('r p y')	
	    ROT_x = Matrix([[ 1,              0,        0],
                    	   [ 0,         cos(r),  -sin(r)],
                    	   [ 0,         sin(r),  cos(r)]])

    	    ROT_y = Matrix([[ cos(p),        0,  sin(p)],
                    	   [      0,        1,       0],
                    	   [-sin(p),        0, cos(p)]])

    	    ROT_z = Matrix([[ cos(y),  -sin(y),       0],
                    	   [ sin(y),   cos(y),       0],
                    	   [      0,        0,       1]])

    	    ROT_EE = ROT_z * ROT_y * ROT_x

    	    Rot_Error = ROT_z.subs(y, radians(180))* ROT_y.subs(p, radians(-90))
    	    ROT_EE = ROT_EE * Rot_Error
    	    ROT_EE = ROT_EE.subs ({'r':roll, 'p' : pitch, 'y': yaw})
    	    EE = Matrix ([[px],
                          [py],
                          [pz]])
            WC = EE - (0.303) * ROT_EE[:,2]
	    # Calculate joint angles using Geometric IK method
	    #
	  
	    theta1 = atan2(WC[1], WC[0])
	    # Triangle sides to apply cosines law
	    l_a = 1.500971685
	    l_b = sqrt(pow((sqrt(WC[0]**2+WC[1]**2)-0.35),2)+pow((WC[2]-0.75),2))
	    l_c = 1.25
	    # Cosines Law
	    angle_a = acos((l_b * l_b + l_c * l_c - l_a * l_a)/( 2 * l_b*l_c))
	    angle_b = acos((l_a * l_a + l_c * l_c - l_b * l_b)/( 2 * l_a*l_c))
	    angle_c = acos((l_a * l_a + l_b * l_b - l_c * l_c)/( 2 * l_a*l_b))

	    theta2 = pi / 2 - angle_a - atan2(WC[2]-0.75, sqrt(WC[0]**2+WC[1]**2)-0.35)
	    theta3 = pi / 2 - (angle_b + 0.03598446)

	    R0_3 = R0_1 * R1_2 * R2_3    
	    R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2 , q3: theta3})
	    R3_6 = R0_3.inv("LU") * ROT_EE

	    # The rest of the angles is calculated by extracting euler angles
	    theta4 = atan2(R3_6[2,2], -R3_6[0,2])
	    theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
	    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

	    FK = T0_EE.evalf(subs = {q1: theta1,q2: theta2,q3: theta3,q4: theta4,q5: theta5,q6: theta6 }
	    your_ee = [FK[0,3],FK[1,3],FK[2,3]] # <--- Load your calculated end effector value from your forward kinematics


	    if not(sum(your_ee)==3):
		ee_x_e = abs(your_ee[0]-test_case[0][0][0])
		ee_y_e = abs(your_ee[1]-test_case[0][0][1])
		ee_z_e = abs(your_ee[2]-test_case[0][0][2])
		ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
		print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
		print ("End effector error for y position is: %04.8f" % ee_y_e)
		print ("End effector error for z position is: %04.8f" % ee_z_e)
		print ("Overall end effector offset is: %04.8f units \n" % ee_offset)

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
