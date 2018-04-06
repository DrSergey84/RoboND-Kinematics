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

![Picture 1](./pictures/DH_table.png)

DH table is obtained basing on the below forward kinematics picture and kr210.urdf.xacro file:

![Picture 2](./pictures/forward_kinematics.png)


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

θ<sub>1</sub> is an angle betwwen X<sub>0</sub> and X<sub>1</sub> measured over Z<sub>1</sub>. Note that when joint 1  X axis is rotated by construction  WC will be rigidly rotated with it. Below picture is illustrating θ<sub>1</sub> calculation approach:

![Picture 3](./pictures/theta1.jpg)

**Note that joints enumeration on the picture above is wrong, it should be 0,1,2,3,4 as the XY axises obviously belomg to the base frame ( I missed one node there). But it doesn't change the idea behind it in anyways**.

θ<sub>2</sub> is an angle between X<sub>1</sub> and X<sub>2</sub> measured over Z<sub>2</sub>. On the picture below θ<sub>2</sub> is shown on the X<sub>2</sub>Y<sub>2</sub> plane, from where θ<sub>2</sub> = pi/2 - a - atan(x,y) ,where (x,y) are the coordinates of the point of WC projected on that plane. Now let's make a simple observation that frame2 origin has a fixed offset (a<sub>1</sub>, d<sub>1</sub>) from frame 0 in frame0's coordinate system.  From the previous picture where θ<sub>1</sub> was drawn we can figure that X<sub>1</sub> is projected on hepotenus of a triangle with the sides equal to (x<sub>c</sub>, y<sub>c</sub>) which makes it to be sqrt(xc^2 + yc^2), or expressed in terms of the base frame it is sqrt(x<sub>c</sub>^2 + y</sub>c</sub>^2) - a1, but on X<sub>0</sub>Y<sub>0</sub> plane X<sub>1</sub> is collinear with Y<sub>2</sub> as it follows from the  forward kinematics picture, thus y = sqrt(x<sub>c</sub>^2 + y<sub>c</sub>^2) - a<sub>1</sub> . Also, since the projection is done along Z axis, x coordinate in terms of the base frame  would be x = z<sub>c</sub> - d<sub>1</sub>.  


![Theta2 and Theta3 image](./pictures/l21-l-inverse-kinematics-new-design-fixed.png)

![Picture 4](./pictures/theta3.png)

θ<sub>3</sub> is an angle between X<sub>2</sub> and X<sub>3</sub> measured over Z<sub>3</sub>. From the picture above we can conclude that θ<sub>3</sub> = (b - d), where d = atan2(d4,a3). 

On the image above the angles *a* , *b* and *c* can be obtained using a cosine theorem, for that we need to figure first *A*, *B* and *C* values for the triangle. *C* is a distance from link 2 to link 3 which can be obtained from DH table (a2 parameter which is a distance from Z<sub>2</sub>  to Z<sub>3</sub> measured over X<sub>2</sub>) as it is shown on the forward kinematics picture above.

In order to figure *A* we can look at the "forward kinematics" image above , where WC is O4,O5,O6. d4 and a3  there define the sides of a right triangle and from there we can figure the *A*. d4 is the distance from X3 to X4 over Z4 and a3 is the distance from Z3 to Z4 over X3.

In order to figure out *B* we can build another right triangle.  sides by substracting d1 from WCz ( d is a distance between Xs measured along Z) and a1 from the projection of the WC on ZY plane.


For finding θ<sub>4</sub>, θ<sub>5</sub> and θ<sub>6</sub>  let's see how R<sub>3,6</sub> matrix looks like

```
R3_6 = Matrix([ [-sin(theta4)*sin(theta6) + cos(theta4)*cos(theta5)*cos(theta6), -sin(theta4)*cos(theta6) - sin(theta6)*cos(theta4)*cos(theta5), -sin(theta5)*cos(theta4)], 
                [sin(theta5)*cos(theta6),                                        -sin(theta5)*sin(theta6),                                       cos(theta5),              ], 
                [-sin(theta4)*cos(theta5)*cos(theta6) - sin(theta6)*cos(theta4), sin(theta4)*sin(theta6)*cos(theta5) - cos(theta4)*cos(theta6), sin(theta4)*sin(theta5)] ])
```


From above we can observe that R<sub>2,2</sub>/-R<sub>0,2</sub> = tan(theta4) or equivalently theta4 = atan2(R<sub>2,2</sub>, -R<sub>0,2</sub>)
Similarly -R<sub>1,1</sub>/R<sub>1,0</sub> = tan(theta6) and equivalently theta6 = atan2(-R<sub>1,1</sub>, R<sub>1,0</sub>)
Now, when we known theta4 and theta6,  sin(theta4) is a known value (f.ex. C) we denote that R<sub>2,2</sub>/R<sub>1,2</sub> = C*tan(theta5), or theta5 = atan2(R<sub>2,2</sub>, C * R<sub>1,2</sub>).

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

 I do not use symbolic computations at all in my code, since they are not really needed. Also I do avoid the forward kinematics part as well as it is not needed either to solve this particular exersice.  I basically need  only R0_3 and
R3_6 as it is seen in the lectures ( see "Inverse Kinematics with Kuka KR210"), thus I use their symbolic expression (pre-computed ) and just substitute there the corresponding angles when needed. 

The handle_calculate_IK function is pretty small and straight-forward. I was basically following the  guide presented under "Inverse Kinematics with Kuka KR210" chapter.

![Pick&Place](./pictures/snapshot1.jpg)
![Pick&Place](./pictures/snapshot2.jpg)
![Pick&Place](./pictures/snapshot3.jpg)
