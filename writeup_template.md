## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---
### Useful links
[Pick and Place Youtube Video](https://www.youtube.com/watch?v=7BWwNKVsJX4)

[Final Code](IK_server.py)

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
[DH-algorithm]: ./misc_images/DH-algorithm.png
[final-result]: ./misc_images/final-result.png
[second-result]: ./misc_images/second-result.png
[individual-matrices]: ./misc_images/individual-matrices.png
[theta1]: ./misc_images/theta1_image.jpeg
[theta2]: ./misc_images/theta2_image.jpeg
[theta3]: ./misc_images/theta3-d.png
[triangle]: ./misc_images/triangle_formula.jpeg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis and Project Implementation
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
* In the kr210.urdf.xacro we have the following values:

* * Joint0-1 --> (0,     0,    0.33)
* * Joint1-2 --> (0.35,  0,    0.42)
* * Joint2-3 --> (0,     0,    1.25)
* * Joint3-4 --> (0.96,  0,  -0.054)
* * Joint4-5 --> (0.54,  0,    0)
* * Joint5-6 --> (0.193, 0,    0)
* * Joint6-EE --> (0.11, 0,    0)

* Then I obtained the following diagram following the process in lesson 11-13 "DH Paremeter Assignment Algorithm"

![alt text][DH-algorithm]
![alt text][FK-image]
![alt text][DH-definitions]

* Finally, we have: 
* * d1 = 0.33 + 0.42 = 0.35
* * a1 = 0.35
* * a2 = 1.25
* * a3 = -0.054
* * d4 = 0.96 + 0.54 = 1.5
* * dG = 0.193 + 0.11 = 0.303


* From the image above, I obtained  a DH paremeter table according to the definitions.


Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.


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
* Then I defined individual transform matrix :

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
* Finally, I have the following matrices :

![alt text][individual-matrices]

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
* To calculate theta3, we know that is the angle between X3 and X4 around Z4. X4 is perpendicular to Z4 wich is aligned to d4. That is because I draw a perpendicular line from JOINT3 to d4 prolongation that is 'a3' . We assume that theta3 moves certain angle (JOINT4 stays  with the same angle) that moves link4. So we can notice that there is an angle from X3 to X4. 
* We prolong d4 and draw a dotted line parallel to it. 
* We build a triangle formed bt JOINT2, JOINT3 and WC.  We notice that 90 + theta3 is equal to the sum of the angles formed by J2,J3,WC and H,d4,a3. Then, similar to theta2, we can calculate 'c' angle by Cosines Law. For gamma, we calculate by atan2(d4,a3) = 0.03598446 in radians. 
* Finally is possible to obtain theta3 as you can see in the following code

![alt text][theta3]
```python
angle_b = acos((l_a * l_a + l_c * l_c - l_b * l_b)/( 2 * l_a*l_c))
theta3 = pi / 2 - (angle_b + 0.03598446)
```
* Finally, we need to calculate the rest of the angles by obtaining R3_6 and extracting euler angles. This code is similar to the lesson 11-8. Also, from lesson 11-5 we can see the atan2 help us to solve the ambiguity to know in wich quadrant is the angle. 
* Theta5 gets 2 values, so I implement a condition for both situations that is very useful. This was found in slack.
```python
R0_3 = R0_1 * R1_2 * R2_3    
R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2 , q3: theta3})
R3_6 = R0_3.inv("LU") * ROT_EE

# Extracting euler angles
theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
	    
if sin(theta5) < 0:
	theta4 = atan2(-R3_6[2,2], R3_6[0,2]
        theta6 = atan2(R3_6[1,1], -R3_6[1,0])
else:
	theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
### Results
#### 1. Discuss the code you implemented and your results. 
* The robot arm is able to complete 8/10 pick and place cycles as you can see in the results. I am not able to show take a photo inside the bin because when I can't manipulate Gazebo (It got crashed), but you can test the code. Maybe it fails once.
![alt text][final-result]
![alt text][second-result]
* A good idea is to perform forward kinematics out of the main loop.
* I don't work with simplify function in matrices as a suggestion in lessons. 
* I think that the arm moves very slow, because of the graphics requierements to run the Gazebo simulation. So this must be improved. 
















