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
import time

def HomTransf_ij(alpha_i, a_i, d_j, q_j):
    
    T_ij = Matrix([[             cos(q_j),               -sin(q_j),             0,                 a_i],
                   [sin(q_j)*cos(alpha_i), cos(q_j) * cos(alpha_i), -sin(alpha_i), -sin(alpha_i) * d_j],
                   [sin(q_j)*sin(alpha_i), cos(q_j) * sin(alpha_i),  cos(alpha_i),  cos(alpha_i) * d_j],
                   [                    0,                       0,             0,                   1],
        ])
    return T_ij


def clip_angle(angle, joint_angle_limit):
    lower_limit = joint_angle_limit[0] * pi/180
    upper_limit = joint_angle_limit[1] * pi/180
    if angle < lower_limit:
        return lower_limit
    elif angle > upper_limit:
        return upper_limit
    else:
        return angle


def Rot(angle, axis):
    if axis == 'x':
        return Matrix([[ 1, 0, 0],
                  [ 0, cos(angle), -sin(angle)],
                  [ 0, sin(angle), cos(angle)]])
    elif axis == 'y':
        return Matrix([[ cos(angle), 0, sin(angle)],
                  [ 0,  1, 0],
                  [ -sin(angle), 0, cos(angle)]])
    elif axis == 'z':
        return Matrix([[ cos(angle),  -sin(angle), 0],
                  [ sin(angle), cos(angle), 0],
                  [ 0,  0, 1]])

def validate(curr_angle, prev_angle):
    if abs(curr_angle - prev_angle) <= abs(-curr_angle - prev_angle):
        #new current angle is in same quadrant as it was in previous step
        return curr_angle
    elif abs(curr_angle - prev_angle) > abs(-curr_angle - prev_angle):
        #curr angle is opposite quadrant
        return -curr_angle




def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint variable theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #link offset dist(x_{i-1}, x_{i})
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #link length dist(z_{i-1}, z_{i})
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #twist angle ang <z_{i-1}, z_{i}>  
      
        # Create Modified DH parameters
        # Create Modified DH parameters
        DH_table = {alpha0: 0,      a0: 0,      d1: 0.75, q1:q1,
                    alpha1: -pi/2., a1: 0.35,   d2: 0,    q2: q2 - pi/2.,
                    alpha2: 0,      a2: 1.25,   d3: 0,     q3: q3, 
                    alpha3: -pi/2., a3: -0.054, d4: 1.50, q4: q4,
                    alpha4:  pi/2., a4: 0,      d5: 0,    q5: q5,
                    alpha5: -pi/2., a5: 0,      d6: 0,    q6: q6,
                    alpha6: 0,      a6: 0,      d7: 0.303,q7: 0}

        joint_angle_limit_theta1 = [-185, 185]
        joint_angle_limit_theta2 = [-45, 85]
        joint_angle_limit_theta3 = [-210, 65]
        joint_angle_limit_theta4 = [-350, 350]
        joint_angle_limit_theta5 = [-125, 125]
        joint_angle_limit_theta6 = [-350, 250]

        # Define Modified DH Transformation matrix
        T0_1 = HomTransf_ij(alpha0, a0, d1, q1) #Transform of frame1 wrt frame0
        T0_1 = T0_1.subs(DH_table)
        
        T1_2 = HomTransf_ij(alpha1, a1, d2, q2)
        T1_2 = T1_2.subs(DH_table)

        T2_3 = HomTransf_ij(alpha2, a2, d3, q3)
        T2_3 = T2_3.subs(DH_table)

        T3_4 = HomTransf_ij(alpha3, a3, d4, q4)
        T3_4 = T3_4.subs(DH_table)

        T4_5 = HomTransf_ij(alpha4, a4, d5, q5)
        T4_5 = T4_5.subs(DH_table)

        T5_6 = HomTransf_ij(alpha5, a5, d6, q6)
        T5_6 = T5_6.subs(DH_table)

        T6_G = HomTransf_ij(alpha6, a6, d7, q7)
        T6_G = T6_G.subs(DH_table)
        #Post-multiplication: Forward kinematic
        T0_2 = T0_1 * T1_2 #transform base link frame to link2 frame
        T0_3 = T0_2 * T2_3 #transform base link to link3
        T0_4 = T0_3 * T3_4 #transform base link to link4
        T0_5 = T0_4 * T4_5 #transform base link to link5
        T0_6 = T0_5 * T5_6 #transform base link to link6
        T0_G = T0_6 * T6_G #transform base link to Gripper
        # Initialize service response
        joint_trajectory_list = []

        # create Gripper rotation matrix:
        r, p, y = symbols('r p y')
        R_G = Rot(y, axis='z') * Rot(p, axis='y') *  Rot(r, axis='x')  #in urdf frame
        #rotation correction to align urdf frame with DH frame  -- works also to align DH frame with urdf
        p_corr, y_corr = symbols('p_corr y_corr')
        R_corr = Rot(y_corr, axis='z') * Rot(p_corr, axis='y')
        R_corr = R_corr.subs({'p_corr': -pi/2, 'y_corr': pi})
        results = "step q1 q2 q3 q4 q5 q6\n"

        for step_idx, x in enumerate(xrange(0, len(req.poses))):
            print("Trajectory step #", str(step_idx))
            
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
            position_G = Matrix([[px],[py],[pz]])

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here 
            #Evaluation of Gripper orientation in urdf frame
            R_G = R_G.subs({'r': roll, 'p': pitch, 'y': yaw})

            #Apply correction
            R_G = R_G * R_corr 
            
            #Calculate wrist center position
            dG= 0.303 #distance (WC, O_G)=d7 in DH table
            WC = position_G - dG *R_G[:,2] #last term is projection of z axis of Gripper frame on axis of reference frame zero
            
            theta1 = atan2(WC[1], WC[0]).evalf()
            #theta1 = clip_angle(theta1, joint_angle_limit_theta1)
            side_a = 1.501
            side_b = sqrt( (sqrt(WC[0]**2 + WC[1]**2) -0.35)**2 + (WC[2]-0.75)**2 )
            side_c = 1.25

            #use cosine laws
            cos_a = (side_b**2 + side_c**2 - side_a**2)/(2*side_b * side_c)
            #sin_a = (1 - cos_a * cos_a)**0.5 #there are 2 colutions +/- sin_a. Chose +sin_a
            #angle_a = atan2(sin_a, cos_a)
            angle_a = acos(cos_a) #there are 2 solutions pos or neg. (cos(a) = cos(-a))
            cos_b = (side_a**2 + side_c**2 - side_b**2)/(2*side_a * side_c)
            #sin_b = (1- cos_b * cos_b)**0.5 # there are 2 solutions +/- . Arbitraily select +sin_b
            #angle_b = atan2( sin_b, cos_b )
            angle_b = acos( cos_b )  #2 possible solution (cos(a) = cos(-a))  
            theta2 = pi/2. - angle_a - atan2( WC[2]-0.75, sqrt(WC[0]**2 + WC[1]**2)-0.35)
            theta2 = theta2.evalf()
            #theta2 = clip_angle(theta2, joint_angle_limit_theta2)
            theta3 = pi/2. - (angle_b + 0.036)
            theta3 = theta3.evalf()
            #theta3 = clip_angle(theta3, joint_angle_limit_theta3)

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            #
            if step_idx == 0:
                prev_theta1, prev_angle_a, prev_angle_b = theta1, angle_a, angle_b
            else:
                #validate angles 
                angle_a = validate(angle_a, prev_angle_a)
                angle_b = validate(angle_b, prev_angle_b)
                prev_theta1, prev_angle_a, prev_angle_b = theta1, angle_a, angle_b
                #recompute theta2 and theta3
                theta2 = pi/2. - angle_a - atan2( WC[2]-0.75, sqrt(WC[0]**2 + WC[1]**2)-0.35)
                theta2 = theta2.evalf()
                #theta2 = clip_angle(theta2, joint_angle_limit_theta2)
                theta3 = pi/2. - (angle_b + 0.036)
                theta3 = theta3.evalf()
                
            #Get rotation matrix of frame 3 wrt frame 0
            R0_3 = T0_3[0:3, 0:3] #extract from Homogeneous transf matrix, the matrix components related to orientation (3x3 matrix) 
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3:theta3})

            R3_6 = R0_3.inv("LU") * R_G
 
            #Euler angles from rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf()
            #theta4 = clip_angle(theta4, joint_angle_limit_theta4)
            theta5 = atan2( sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2]).evalf() #at least 2 solutions
            #if step_idx == 0:
            #    prev_theta5 = theta5
            #else:
                #validate theta5
            #    test_theta5 = atan2( -sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2]).evalf()
            #    if abs(theta5 - prev_theta5) > abs(test_theta5 - prev_theta5):
            #        theta5 = test_theta5

            #theta5 = clip_angle(theta4, joint_angle_limit_theta5)
            theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf()
            #theta6 = clip_angle(theta4, joint_angle_limit_theta6)

            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
            print(" ***Angles: {} | {} | {} | {} | {} | {}".format( degrees(theta1), degrees(theta2), \
                degrees(theta3), degrees(theta4), \
                degrees(theta5), degrees(theta6) ) )
            results += str(step_idx) +" "+ str(degrees(theta1)) +" "+ str(degrees(theta2)) +" "+ str(degrees(theta3)) +" "+ str(degrees(theta4)) +" "+ str(degrees(theta5)) +" "+ str(degrees(theta6)) +"\n"
            #Compute the forward Kinematic
            
            T_FK = T0_G.evalf(subs={q1:theta1, q2: theta2, q3: theta3, q4: theta4, q5:theta5, q6:theta6 })

        with open("IK_angles_per_step.txt", "w") as text_file:
            text_file.write(results)

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
