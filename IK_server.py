#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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
from numpy import *


# c^2 = a ^2 + b^2 - 2*a*b*cos(alpha)
# cos(alpha) = -(c^2 - a^2 - b^2)/ (2*a*b)
# cos(alpha) = (a^2 + b^2 - c^2) / (2*a*b)
# alpha = atan2( sin(alpha), cos(alpha) )
def angle_from_cosine_theorem(a, b, c):
  cos_alpha = (a*a + b*b - c*c) / (2*a*b)
  sin_alpha = sqrt( 1.0 - cos_alpha**2)
  return arctan2( sin_alpha, cos_alpha )

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

	# NOTE: I do not use symbolic computations at all, since they are not really needed. Also I do avoid all the forward kinematics
	# part as well as it is not needed either to solve this particular exersice.  I basically need  only R0_3 and
	# R3_6 as it is seen in the lectures ( see "Inverse Kinematics with Kuka KR210"), thus I use their symbolic expression (pre-computed )
	# and use them directly substituted with the agle values accordingly.

	# Extract rotation matrices from the transformation matrices
	# alpha0 = 0, a0 = 0, d1 = 0.75 
	#T0_1 = Matrix([ [ cos(theta1), -sin(theta1), 0, 0],
	#                [ sin(theta1), cos(theta1),  0, 0],
	#                [ 0,           0,            1, 0.75],
	#                [ 0,           0,            0, 1]])
	# alpha1 = -90, a1 = 0.35, d2 = 0
	#T1_2 = Matrix([ [ cos(theta2), -sin(theta2), 0, 0.35],
	#                [ 0,            0,           1, 0],
	#                [ -sin(theta2), -cos(theta2),0, 0],
	#                [ 0,           0,            0, 1]])
	# alpha0 = 0, a2 = 1.25, d3 = 0
	#T2_3 = Matrix([ [ cos(theta3), -sin(theta3), 0, 1.25],
	#                [ sin(theta3), cos(theta3),  0, 0],
	#                [ 0,           0,            1, 0],
	#                [ 0,           0,            0, 1]])
        # So this means that R0_1*R1_2*R2_3 gives us
	#R0_3 = Matrix([ [sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2), -sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3), -sin(q1)], 
        #                [sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2), -sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3), cos(q1)], 
        #                [-sin(q2)*sin(q3) + cos(q2)*cos(q3), -sin(q2)*cos(q3) - sin(q3)*cos(q2), 0]])

	# alpha3 = -90
	#R3_4 = Matrix([ [ cos(theta4), -sin(theta4), 0, 0],
	#                [ 0,           0,            1, 0],
	#                [ -sin(theta4),-cos(theta4), 0, 0],
	#                [ 0,           0,            0, 1]])
	#
	# alpha4 = 90
	#R4_5 = Matrix([ [ cos(theta5), -sin(theta5), 0, 0],
	#                [ 0,           0,           -1, 0],
	#                [ sin(theta5), cos(theta5),  0, 0],
	#                [ 0,           0,            0, 1]])
	#
	# alpha5 = -90
	#R5_6 = Matrix([ [ cos(theta6), -sin(theta6), 0, 0],
	#                [ 0,           0,            1, 0],
	#                [ -sin(theta6), -cos(theta6),  0, 0],
	#                [ 0,           0,            0, 1]])
	# So R3_6 is then 
	#R3_6 = Matrix([ [-sin(theta4)*sin(theta6) + cos(theta4)*cos(theta5)*cos(theta6), -sin(theta4)*cos(theta6) - sin(theta6)*cos(theta4)*cos(theta5), -sin(theta5)*cos(theta4), 0], 
	#                [sin(theta5)*cos(theta6),                                        -sin(theta5)*sin(theta6),                                       cos(theta5),              0], 
	#                [-sin(theta4)*cos(theta5)*cos(theta6) - sin(theta6)*cos(theta4), sin(theta4)*sin(theta6)*cos(theta5) - cos(theta4)*cos(theta6), sin(theta4)*sin(theta5),   0], 
	#                [0,                                                              0,                                                             0,                         1]])
	# From above we can observe that R2_2/R0_2 = -tan(theta4) and equivalently theta4 = atan2(R2_2, -R0_2)
	# Similarly                     R1_1/R1_0 = -tan(theta6) and equivalently theta6 = atan2(-R1_1, R1_0)
	# Now, when we known theta4 and theta6  sin(theta4) is a known value (f.ex. C), which means R2_2/R1_2 = C*tan(theta5), or theta5 = atan2(R2_2, C * R1_2), the sign of sin(theta4)
	# matters though, so I need to take that into account in the code.

	#URDF/DH Rotation discrepancy compensation matrices
	#T_z = Matrix([ [cos(np.pi), -sin(np.pi), 0, 0],
	#               [sin(np.pi), cos(np.pi), 0, 0],
	#               [0,          0,          1, 0],
	#               ]0,          0,          0, 1]])
	#T_y = Matrix([ [cos(-np.pi/2), 0, sin(-np.pi/2), 0],
	#               [0,             1, 0,             0],
	#               [-sin(-np.pi/2),0, cos(-np.pi/2), 0],
	#               ]0,             0,       0,       1]])
	# Below is the result of R_z * Ry multiplication of the matrices above
	#R_corr = matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])

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

	    # Define Rrpy rotation matrix including a compensation 
	    # for rotation discrepancy between DH parameters and Gazebo
	    #DH_R_x = matrix([[ 1,              0,        0],
	    #                 [ 0,        c_roll, -s_roll],
	    #                 [ 0,        s_roll,  c_roll]])

	    #DH_R_y = matrix([[ c_pitch,     0,  s_pitch],
	    #                 [       0,        1,        0],
	    #                 [-s_pitch,     0,  c_pitch]])

	    #DH_R_z = matrix([[ c_yaw, -s_yaw,     0],
	    #                [ s_yaw,  c_yaw,      0],
	    #                [ 0,              0,        1]])

	    #Rrpy = DH_R_z * DH_R_y * DH_R_x * R_corr
	    s_pitch = sin(pitch)
	    c_pitch = cos(pitch)
	    s_roll = sin(roll)
	    c_roll = cos(roll)
	    s_yaw = sin(yaw)
	    c_yaw = cos(yaw)
	    s_pitch_s_roll = s_pitch * s_roll
	    c_roll_c_yaw = c_roll * c_yaw
	    s_yaw_c_roll = s_yaw * c_roll

	    Rrpy = matrix([ [s_pitch*c_roll_c_yaw + s_roll*s_yaw, -s_pitch_s_roll*c_yaw + s_yaw_c_roll, c_pitch*c_yaw], 
	                    [s_pitch*s_yaw_c_roll - s_roll*c_yaw, -s_pitch_s_roll*s_yaw - c_roll_c_yaw, s_yaw*c_pitch], 
	                    [c_pitch*c_roll, -s_roll*c_pitch, -s_pitch] ])

	    # Get the end reflector Z vector
	    nx = Rrpy[0, 2]
	    ny = Rrpy[1, 2]
	    nz = Rrpy[2, 2]
            # Get wrist center, d6 is 0 due to the frames chosen
            # end reflector's length is 0.303m according to the DH values shown in the video
	    # See "RK210 Forward kinematics 3"  in the lectures
	    d6 = 0; l = 0.303
	    wx = px - (d6 + l)*nx
	    wy = py - (d6 + l)*ny
	    wz = pz - (d6 + l)*nz
	    #
	    # Calculate joint angles using Geometric IK method
	    theta1 = arctan2(wy,wx)

	    link_2_3 = 1.25 # link 2 to 3 distance
	    link_3_wc = sqrt(1.5**2 + (-0.054)**2) # link3 to wc distance
	    z_pos = wz - 0.75
	    y_pos = sqrt( wx*wx + wy*wy ) - 0.35
	    link_2_wc = sqrt(pow(y_pos, 2) +  pow(z_pos,2)) 
	    a = angle_from_cosine_theorem( link_2_3, link_2_wc, link_3_wc)
	    b = angle_from_cosine_theorem( link_2_3, link_3_wc, link_2_wc)
	    c = angle_from_cosine_theorem( link_2_wc, link_3_wc, link_2_3)
	    theta2 = pi / 2 - a - arctan2(z_pos, y_pos) 
	    theta3 = -(b - arctan2(1.5, 0.054)) 

	    #Pre-compute sin/cos values of thetas 1,2 and 3. Not sure if python is a smart enough to do it by itself
	    c_t1 = cos(theta1)
	    c_t2 = cos(theta2)
	    c_t3 = cos(theta3)
	    s_t1 = sin(theta1)
	    s_t2 = sin(theta2)
	    s_t3 = sin(theta3)
	    c_t2_t3 = c_t2 * c_t3
	    s_t2_t3 = s_t2 * s_t3
	    s_t2_c_t3 = s_t2 * c_t3
	    # Since rotation matrix is orthogonal it's inverse is equal to it's transpose
	    R0_3_inv = matrix([ [s_t2_c_t3*c_t1 + s_t3*c_t1*c_t2, -s_t2_t3*c_t1 + c_t1*c_t2_t3, -s_t1],
                            [s_t1*s_t2_c_t3 + s_t1*s_t3*c_t2, -s_t1*s_t2_t3 + s_t1*c_t2_t3, c_t1],
                            [-s_t2_t3 + c_t2_t3, -s_t2*c_t3 - s_t3*c_t2, 0]]).getT()

	    R3_6 = R0_3_inv * Rrpy
	    #Now get the joint 4,5,6 parameters
	    theta4 = arctan2(R3_6[2,2], -R3_6[0,2])
	    theta6 = arctan2(-R3_6[1,1], R3_6[1,0])
	    #The effector doesn't take the right position when sin(theta4) is negative without this
	    # adjustment. For inst. it sometimes cannot  drop a can into the bin properly due 
	    # to the wrong orientation of the gripper.
	    s_theta4 = sin(theta4)
	    if (s_theta4 > 0 ):
	        theta5 = arctan2(R3_6[2,2], s_theta4 * R3_6[1,2])
	    else:
	        theta5 = arctan2(-R3_6[2,2], s_theta4 * R3_6[1,2])

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
