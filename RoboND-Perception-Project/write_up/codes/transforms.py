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

def HomTransf_ij(alpha_i, a_i, d_j, q_j):
    
    T_ij = Matrix([[             cos(q_j),               -sin(q_j),             0,                 a_i],
                   [sin(q_j)*cos(alpha_i), cos(q_j) * cos(alpha_i), -sin(alpha_i), -sin(alpha_i) * d_j],
                   [sin(q_j)*sin(alpha_i), cos(q_j) * sin(alpha_i),  cos(alpha_i),  cos(alpha_i) * d_j],
                   [                    0,                       0,             0,                   1],
        ])
    return T_ij

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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        #joint variable theta_i
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') 
        #link offset dist(x_{i-1}, x_{i})
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        #link length dist(z_{i-1}, z_{i})
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        #twist angle ang <z_{i-1}, z_{i}>  
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
      
        # Create Modified DH parameters
        # Create Modified DH parameters
        DH_table = {alpha0: 0, a0: 0, d1: 0.75, q1:q1,
            alpha1: -pi/2., a1: 0.35, d2: 0, q2: q2 - pi/2.,
            alpha2: 0, a2: 1.25, d3:0, q3: q3, 
            alpha3: -pi/2., a3: -0.054, d4: 1.50, q4: q4,
            alpha4:  pi/2., a4: 0, d5: 0, q5: q5,
            alpha5: -pi/2., a5: 0, d6: 0, q6: q6,
            alpha6: 0, a6: 0, d7: 0.303, q7: 0}  
        # Define Modified DH Transformation matrix
        #Transform of frame1 wrt frame0
        T0_1 = HomTransf_ij(alpha0, a0, d1, q1) 
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
        #transform base link frame to link2 frame
        T0_2 = simplify(T0_1 * T1_2)
        #transform base link to link3
        T0_3 = simplify(T0_2 * T2_3)
        #transform base link to link4
        T0_4 = simplify(T0_3 * T3_4)
        #transform base link to link5
        T0_5 = simplify(T0_4 * T4_5) 
        #transform base link to link6
        T0_6 = simplify(T0_5 * T5_6) 
        #transform base link to Gripper
        T0_G = simplify(T0_6 * T6_G) 

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
            position_G = Matrix([[px],[py],[pz]])

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
            # create Gripper rotation matrix:
            r, p, y = symbols('r p y')
            rot_G = Rot(r, axis='x') * Rot(p, axis='y') * Rot(y, axis='z')
            #rotation correction to align Gripper link frame 
            #(as defined in urdf) to frame with DH convention
            p_corr, y_corr = symbols('p_corr y_corr')
            R_corr = Rot(y_corr, axis='z') * Rot(p_corr, axis='y')
            #Apply correction
            rot_G = rot_G * R_corr
            #Evaluation of Gripper orientation
            rot_G = rot_G.subs(
            {'r': roll, 'p': pitch, 'y': yaw, 'p_corr': radians(-90), 'y_corr': radians(180)})
    
            
            #Calculate wrist center position
            dG= 0.303 #distance (O_5, O_G) given by urdf
             #last term is projection of z axis of 
             #Gripper frame on axis of reference frame zero
            WC = position_G - dG * rot_G[:,2]
            
            theta1 = atan2(WC[1], WC[0])
            side_a = 1.501
            side_b = sqrt((sqrt(WC[0]**2 + WC[1]**2)-0.35)**2 + (WC[2]-0.75)**2)
            side_c = 1.25

            #use cosine laws 
            angle_a = acos((side_b**2 + side_c**2 - side_a**2)/(2*side_b * side_c))
            angle_b = acos((side_a**2 + side_c**2 - side_b**2)/(2*side_a * side_c))
            angle_c = acos((side_a**2 + side_b**2 - side_c**2)/(2*side_a * side_b))

            theta2 = pi/2. - angle_a - atan2(WC[2]-0.75, sqrt(WC[0]**2 + WC[1]**2)-0.35)
            theta3 = pi/2. - (angle_b + 0.036)

            #Get rotation matrix of frame 3 wrt frame 0
            #extract from Homogeneous transf matrix, 
            #the components related to orientation (3x3 matrix) 
            R0_3 = T0_3[0:3, 0:3] 
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3:theta3})
            # R3_6 = inv(R0_3) * R0_G (R0_G = rot_G)
            R3_6 = R0_3.inv("LU") * rot_G

            #Euler angles from rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = 
                [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            #Compute the forward Kinematic 
            FK = T0_G.evalf(subs=
                {q1:theta1, q2: theta2, q3: theta3, q4: theta4, q5:theta5, q6:theta6 })
            #print error orientation, position: sqrt( (FK - rot_G)**2)
            #err_orientation = np.sqrt( (FK[:,0:3] - rot_G) * (FK[:,0:3] - rot_G) )
            #print(err_orientation)
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
