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

Since we have 2 translational and 2 rotational variables for each joint, it is helpful to make a generalized homogeneous trasform.
<HERE>

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


