## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[image_sketch_1]: ./misc_images/sketch1.JPG
[image_sketch_2]: ./misc_images/sketch2.JPG
[image_sketch_3]: ./misc_images/sketch3.JPG
[image_sketch_4]: ./misc_images/sketch4.JPG
[Project2Lesson15_IMG1]: ./misc_images/Project2Lesson15_IMG1.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! This file is base on the writeup that was provided by the course.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To begin, I needed to know the zero position of the robot arm.
![alt_text][image1]

For the kinematics of the KR210, I made a sketch of the robot's joints at there zeros position.

![alt text][image_sketch_1]

This sketch is based on the begining sketch that was presented in the lesson KR210 Forward Kinematics 1. Using this as my baseline, I added the axes of movement and numbered the links. Note that G would be rebresented as EE in the code.

![alt text][image_sketch_2]

Next, I wrote down the descriptions for the a's, d's, alphas, and q's.

![alt text][image_sketch_3]

Note that I also added the global coordinate system to the skecth. The sketch was helpful in understanding how to map a robotic arm.This made it easier to interpret the URDF file. For istance to get the value of d1,I knew that I needed to add the Z components of joint 1 (0.33) and joint 2 (0.42). Using the URDF file, I looked at the values for the dimensions of the KR210.

#### 2. Using the DH parameters derived earlier, I created the following DH parameter table

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.33+0.42 =0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | q2-pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054| 0.96+0.54=1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.193+0.11=0.303 | 0

Since we have 2 translational and 2 rotational variables for each joint, it is helpful to make a generalized homogeneous trasform.I started with the form described in Lesson 11.12 Equations 2 and 3. To prove that the math worked, I worte a quick python script to develop the homogenous transform, see below

```python
# import modules
import rospy
import tf
from mpmath import *
from sympy import *

 # Generic rotation about x
def rotx(theta):
    R=Matrix([[1 , 0         , 0          ],
              [0 , cos(theta), -sin(theta)],
              [0 , sin(theta), cos(theta) ]])
    return R

# Generic rotation about z
def rotz(theta):
    R=Matrix([[cos(theta), -sin(theta), 0],
              [sin(theta), cos(theta),  0],
              [0         ,          0,  1]])
    return R
    
# Make symbols for making DH Parameter Tables
alpha = symbols('alpha')
a = symbols('a')
d = symbols('d')
q = symbols('q')

# About the X-Axis
HT1=Matrix([rotx(alpha)])
HT1=HT1.col_insert(3,Matrix([a, 0, 0]))
HT1=HT1.row_insert(3,Matrix([[0, 0, 0, 1]]))

# About the Z-Axis
HT2=Matrix([rotz(q)])
HT2=HT2.col_insert(3,Matrix([0, 0, d]))
HT2=HT2.row_insert(3,Matrix([[0, 0, 0, 1]]))

#Combine X then Z
HT=simplify(HT1*HT2)
    
print(HT)
```

In order to save time, in the code that I developed for the IK, I made a function that took the inputs of a, alpha, d, and q(theta). That save the time and space of making all of the trasforms individually. For each link I created the HT table. To get the overall transform, I multiplied the HT of each link together sequencially. I then multiplied by a rotation about the  Z-axis of pi and about the Y-axis of -pi/2 to correct for the wrist orientation. These steps were done prior to the look through all of the poses the maximize speed. The following is the HT table function that I used

```python
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
```

Note that the second version is the same as the first, just alligned with the example.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

For the inverse kinematics, we know that the is a spherical wrist. This simplifies things into two tasks. The first task is to find the theta angles for joints 1-3 to place the wrist in the correct location. The second task is determining the proper theta angles for joints 4-6 to orient the wrist correctly.

In order to calculate the end-effector's rotation matrix, I applied the composite rotations and correction matricies. Then I corrected for the offset in d7.

```python
# Rotation matrix for end-effector
            Rot_EE= rotz(yaw) * roty(pitch) * rotx(roll) * rotGripCorrection
            
          
            # Find the Wrist location (end-effector - the offset caused by d7)       
            w_x = px - 0.303 * Rot_EE[0,2]
            w_y = py - 0.303 * Rot_EE[1,2]
            w_z = pz - 0.303 * Rot_EE[2,2]
```

Now I could slove for thetas 1-3. Thata 1 was the eaisest. Since it is a rotation about the Z-axis, the arctangent of w_x and w_y can be used.
![alt_text][sketch4]

```python
            # Calculate joint angles using Geometric IK method #
            # Start with theta1 = atan2(w_y, w_x)
            theta1= atan2(w_y , w_x)   
```
Theata 2 and 3 are trickier. The following drawing from the lesson (Project 2 Lesson 15)is a good visualization of those angle.

![alt_text][Project2Lesson15_IMG1]

This meant using the law of cosines to solve for the angles, after calculating the sides.

```python
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
```

With the first 3 theta's solved, it was now time to calculate theta 4-6. These are the Euler angles of the end-effector.
The most significant upgrade to this section of code was using the transpose instead of the inverse. This made the code much faster and reliable. That fact was mentioned on slack.

```python
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
'''

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


