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
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/misc3.png
[FK-image]: ./misc_images/FK_image.jpg
[tf-matrix]: ./misc_images/TF_Matrix.png
[wrist-center]: ./misc_images/wrist_center.png
[DH-definitions]: ./misc_images/DH-paremeters-definitions.png
[theta1]: ./misc_images/theta1_image.jpeg
[theta2]: ./misc_images/theta2_image.jpeg
[theta3]: ./misc_images/theta3_image.jpeg
[triangle]: ./misc_images/triangle_formula.jpeg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis and Project Implementation
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

* First I obtained the following diagram and I obtained the variable values from kr210.urdf.xacro .

![alt text][FK-image]
![alt text][DH-definitions]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

* From the image above, I obtained  a DH paremeter table according to the definitions.


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 1.25 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.35 | 0

* To obtain Transformation Matrix, I defined TF_Matrix function:

![alt text][tf-matrix]

```python
def TF_Matrix(alpha, a, d, q):

	TF = Matrix([[                cos(q),            -sin(q),             0,              a],
		     [     sin(q)*cos(alpha),  cos(q)*cos(alpha),   -sin(alpha),  -sin(alpha)*d],
		     [     sin(q)*sin(alpha),  cos(q)*sin(alpha),    cos(alpha),   cos(alpha)*d],
		     [                     0,                  0,             0,             1]])
	return TF
```
Then I defined individual transform matrix :

```python
	T0_1 =  TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
    	T1_2 =  TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
    	T2_3 =  TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
    	T3_4 =  TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
   	T4_5 =  TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
    	T5_6 =  TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
    	T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
	# Extract rotations matrices
	R0_1 = T0_1[0:3,0:3]
	R1_2 = T1_2[0:3,0:3]
	R2_3 = T2_3[0:3,0:3]
```


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

* I obtained the wrist center according to the given formula in lectures.  
![alt text][wrist-center]
* First, we need to calculate the rotation matrix from the base to the EE with the given roll, pitch and yaw values. However, this rotation matrix needs to be rotated around z and y axis.
```python
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
```
* Then, I obtained theta1 with geometric analysis. We can see a triangle formed by a2 , H and D. We calculate D with  wrist center coordinates and d1. 
![alt text][theta1]
```python
theta1 = atan2(WC[1], WC[0])
```
* The next step is to calculate theta2.
![alt text][theta2]

* We can see that theta2=90-a-alpha. To calculate 'alpha' we apply Pithagorean and for 'a' we need Cosines Law. 
![alt text][triangle]

```python
# Define triagle sides to perform Cosines Law.  
l_a = 1.500971685
l_b = sqrt(pow((sqrt(WC[0]**2+WC[1]**2)-0.35),2)+pow((WC[2]-0.75),2))
l_c = 1.25

# WE apply cosines law.  
angle_a = acos((l_b * l_b + l_c * l_c - l_a * l_a)/( 2 * l_b*l_c))
angle_b = acos((l_a * l_a + l_c * l_c - l_b * l_b)/( 2 * l_a*l_c))
angle_c = acos((l_a * l_a + l_b * l_b - l_c * l_c)/( 2 * l_a*l_b))

theta2 = pi / 2 - angle_a - atan2(WC[2]-0.75, sqrt(WC[0]**2+WC[1]**2)-0.35)

```
* Now, we need to calculate theta3 and I think this is the most challenging part. theta3 = 90-(c+gamma), 'c' is calculated by the Cosine
Law in the triangle above. To calculate gamma, you can calculate acos(d4,H) = 0.03598446. You also can see the formula to calculate H and use this value in the Cosine Law.
![alt text][theta3]
```python
theta3 = pi / 2 - (angle_b + 0.03598446)
```
* Finally, we need to calculate the rest of the angles by obtaining R3_6 and extracting euler angles. This code is similar to the lesson 11-8.
```python
R0_3 = R0_1 * R1_2 * R2_3    
R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2 , q3: theta3})
R3_6 = R0_3.inv("LU") * ROT_EE

theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
### Results
#### 1. Discuss the code you implemented and your results. 
* The robot arm is able to complete 8/10 pick and place cycles
* A good idea is to perform forward kinematics out of the main loop.
* I think that the arm moves very slow, because of the graphics requierements to run this simulation. So this must be improved. 

![alt text][image2]















