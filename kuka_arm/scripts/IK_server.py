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
from sympy import *
#from sympy import symbols, cos, sin,  pi, simplify, atan2, acos, sqrt
#from sympy.matrices import Matrix
#import numpy as np


 # Generic rotation about x
def rotx(theta):
    R=Matrix([[1 , 0         , 0          ],
              [0 , cos(theta), -sin(theta)],
              [0 , sin(theta), cos(theta) ]])
    return R

# Generic rotation about y
def roty(theta):
    R=Matrix([[cos(theta) , 0 , sin(theta) ],
              [0          , 1 , 0          ],
              [-sin(theta), 0,  cos(theta) ]])
    return R

# Generic rotation about z
def rotz(theta):
    R=Matrix([[cos(theta), -sin(theta), 0],
              [sin(theta), cos(theta),  0],
              [0         ,          0,  1]])
    return R

## Generic Homogeneous transform table
def HTtable(alpha, a, d, q):
#    table = Matrix([[            cos(q),           -sin(q),           0,             a],
#                    [ sin(q)*cos(alpha), cos(alpha)*cos(q), -sin(alpha), -d*sin(alpha)],
#                    [ sin(alpha)*sin(q), sin(alpha)*cos(q),  cos(alpha),  d*cos(alpha)],
#                    [                 0,                 0,           0,             1]])

    table = Matrix([[            cos(q),           -sin(q),           0,             a],
                    [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                 0,                 0,           0,             1]])                     
    return table        
        

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Make symbols for making DH Parameter Tables
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        
         
        # Create Modified DH parameters #
        s = {alpha0:     0, a0:        0, d1:    0.75, q1: q1,
             alpha1: -pi/2, a1:     0.35, d2:       0, q2: q2-(pi/2.),
             alpha2:     0, a2:     1.25, d3:       0, q3: q3,
             alpha3: -pi/2, a3:  -0.054, d4:     1.50, q4: q4,
             alpha4:  pi/2, a4:        0, d5:       0, q5: q5,
             alpha5: -pi/2, a5:        0, d6:       0, q6: q6,
             alpha6:     0, a6:        0, d7:   0.303, q7: 0}
             
        # Create individual transformation matrices #
        ### Make Homogeneous Transforms between neighboring links
          
        # Base to link1
        T0_1 = HTtable(alpha0, a0, d1, q1).subs(s)
        
        # Link1 to link2
        T1_2 = HTtable(alpha1, a1, d2, q2).subs(s)
           
        # Link2 to link3
        T2_3 = HTtable(alpha2, a2, d3, q3).subs(s)
        
        # Link3 to link4
        T3_4 = HTtable(alpha3, a3, d4, q4).subs(s)
        
        # Link4 to link5
        T4_5 = HTtable(alpha4, a4, d5, q5).subs(s)
        
        # Link5 to link6
        T5_6 = HTtable(alpha5, a5, d6, q6).subs(s)
        
        # Link6 to grip
        T6_g = HTtable(alpha6, a6, d7, q7) .subs(s)
        
        # Overall Trasform
        T0_g = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_g
        
        #Correction factors
        # Compensate for rotation discrepancy between DH parameters and Gazebo #
        # Correction to fix the orientation diff between the gripper in the URDF file and the DH convention
        # rotate about Z by 180 deg (pi rad)
        rotZ = rotz(pi)
    
        # rotate about Y by -90 deg (-pi/2 rad)
        rotY = roty(-pi/2)

        # Calculate the correction matrix        
        rotGripCorrection = rotZ * rotY
        

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and ori./safe_spawner.shentation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            print 'Processing pose: %s' % x

            # Rotation matrix for end-effector
            Rot_EE= rotz(yaw) * roty(pitch) * rotx(roll) * rotGripCorrection
            
          
            # Find the Wrist location (end-effector - the offset caused by d7)       
            w_x = px - 0.303 * Rot_EE[0,2]
            w_y = py - 0.303 * Rot_EE[1,2]
            w_z = pz - 0.303 * Rot_EE[2,2]
            
            
            # Calculate joint angles using Geometric IK method #
            # Start with theta1 = atan2(w_y, w_x)
            theta1= atan2(w_y , w_x)            
            
            # now use law of cosines to get theta 2 and 3. Figure provided in
            # lesson
            
            # Note that this is being solved in the gloabal YZ plane. 
            # Imagine a cordinate system at joint 1 that rotates by theta1.
            # The YZ plane of this coordinate system will contain links 2 and 3.
            # As joint 1 rotates, the z value of the line from 2 to the wrist 
            # will be the same but the Y value(global) will be sqrt(X^2+Y^2). 
            
                
            # define sides (lowercase a, b, c)
            #length from 3 to wrist
            a = 1.5
            
            # We know the wrist distance, and that the Y component is made up 
            # of sqrt(w_x^2 + w_y^2). Don't forget to correct by the Z and Y
            # components from point 2
            b = sqrt(pow((sqrt(w_x**2 + w_y**2) - .35),2) + pow((w_z -.75),2))
            
            #length from 2 to 3
            c = 1.25
            
            # Calc angles (uppercase A, B, C)
            A=acos((-pow(a,2) + pow(b,2) + pow(c,2))/(2*b*c))            
            B=acos(( pow(a,2) - pow(b,2) + pow(c,2))/(2*a*c))
            #C not needed
#           C=acos(( pow(a,2) + pow(b,2) - pow(c,2))/(2*a*b))
            
            # Determine theta2 and theta3
            theta2 = pi / 2 - A - atan2(w_z - 0.75, sqrt(w_x**2 + w_y**2) -0.35)
            theta3 = pi /2 - (B + 0.036)
            
            # Calc Euler angles
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]            
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            
            # R3_6 = R0_3.inv("LU") * Rot_EE
            
            # Slack notes that you can use the traspose since it is the inverse
            # Much faster and more reliable
            R3_6 = R0_3.T * Rot_EE
            
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2),R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            
             
            print '      Done with pose: %s' % x
        
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
