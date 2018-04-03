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

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

it is already done in the videos of the course. :)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

DH table for Kuka210 has been shown in one of the lecture videos (see "KR210 Forward Kinematics 3"). As a lazy guy I have basically used those values. Probably there is no point to duplicate them here.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

![Theta2 and Theta3 image](https://github.com/DrSergey84/RoboND-Kinematics/blob/master/pictures/l21-l-inverse-kinematics-new-design-fixed.png)

On the image above the angles *a* , *b* and *c* can be obtained using a cosine theorem, for that we need to figure first *A*, *B* and *C* values for the triangle. *C* is a distance from link 2 to link 3 which can be obtained from DH table.  In order to figure *A* we can look at the lecture video ( KR210 Forward Kinematics 1, where WC is O4,O5,O6 ). d4 and a3  there define the sides of a right triangle and from there we can figure the *A*.   In order to figure out *B* we  find another right triangle and it's sides by substracting d1 from WCz ( d is a distance between Xs measured along Z) and a1 from the projection of the WC on ZY plane.

For finding Theta3, 4, 5  see the comment section in the beggining of *handle_calculate_IK*  function in ![IK_server](https://github.com/DrSergey84/RoboND-Kinematics/blob/master/IK_server.py)

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

 I do not use symbolic computations at all in my code, since they are not really needed. Also I do avoid the forward kinematics part as well as it is not needed either to solve this particular exersice.  I basically need  only R0_3 and
R3_6 as it is seen in the lectures ( see "Inverse Kinematics with Kuka KR210"), thus I use their symbolic expression (pre-computed ) and just substitute there the corresponding angles when needed. 

The handle_calculate_IK function is pretty small and straight-forward. I was basically following the  guide presented under "Inverse Kinematics with Kuka KR210" chapter.

![Pick&Place](https://github.com/DrSergey84/RoboND-Kinematics/blob/master/pictures/default_gzclient_camera(1)-2018-04-02T12_34_14.500741.jpg)

*The arm has troubles picking the objects ( seems like the ones located on the left from it ). Either I do press 'Next' to early or it is just a wrong math. This needs to be debugged further.*
