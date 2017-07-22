[//]: # (Image References)

[axis_assignment]: ./images/axis_assignment.PNG
[dh_definations]: ./images/DH_angles_determination.PNG
[dh_transformation_equation]:./images/dh_transformation_equation.PNG
[theta3_existing]:./images/theta3_existing.jpg
[theta3_new]:./images/theta3_new.jpg
[wc]:./images/wc.jpg
[theta2]:./images/theta2.jpg
### Axis Assignemnt

Below picture depicts how axis assignemnt is done using DH axis assignement convention

![alt text][axis_assignment]


### DH parameters Tables:

Using DH parameters defination as shown in below image, DH parameters are derived.
To get the orientations of prismatic joints and length of links [kr210.urdf.xacro](https://github.com/udacity/RoboND-Kinematics-Project/blob/master/kuka_arm/urdf/kr210.urdf.xacro) file is referenced.
![alt text][dh_definations]

| i | alpha(i-1) | a (i-1) | d (i) | theta (i)
| --- | --- | --- | --- | --- |
| T_0_1 | 0 | 0.0  | 0.75  | q1  |
| T_1_2 | -pi/2 | 0.35  | 0  | -pi/2 + q2  |
| T_2_3 | 0 | 1.25  | 0  | q3  |
| T_3_4 | -pi/2 | -0.054  | 1.5  | q4  |
| T_4_5 | pi/2 | 0  | 0  | q5  |
| T_5_6 | -pi/2 | 0  | 0  | q6  |
| T_6_G | 0 | 0  | 0.303  | 0 |

### Individual Transformation Matrix.

Once DH parameters are known at each frame, Transformation matrix can be obtained by substituting values in below formula.

![alt text][dh_transformation_equation]

Same is define using python code.

``` python
def get_homogeneous_matrix(a,alpha,d,q):

	T = Matrix([[cos(q),				-sin(q),		0,		a],
		    [sin(q)*cos(alpha), 	cos(q)*cos(alpha),	-sin(alpha), 	-sin(alpha)* d],
		    [sin(q)*sin(alpha),		cos(q)*sin(alpha),	cos(alpha),		cos(alpha) * d],
		    [	0	,			0	,		0,		1]])

	return T

```


THe function returns Matrix which contains symbols. To replace symbols with actual values "subs" keyword in python isused. 


``` python
T0_1 = get_homogeneous_matrix(a0,alpha0,d1,q1)
T1_2 = get_homogeneous_matrix(a1,alpha1,d2,q2)
T2_3 = get_homogeneous_matrix(a2,alpha2,d3,q3)
T3_4 = get_homogeneous_matrix(a3,alpha3,d4,q4)
T4_5 = get_homogeneous_matrix(a4,alpha4,d5,q5)
T5_6 = get_homogeneous_matrix(a5,alpha5,d6,q6)
T6_G = get_homogeneous_matrix(a6,alpha6,d7,q7)


T0_1 = T0_1.subs(s)
T1_2 = T1_2.subs(s)
T2_3 = T2_3.subs(s)
T3_4 = T3_4.subs(s)
T4_5 = T4_5.subs(s)
T5_6 = T5_6.subs(s)
T6_G = T6_G.subs(s)

```
Once DH parameters are substituted, each transformation matrix will have only one known, which is joint angle.
These joint angles are defined by symbols q1,q2,q3,q4,q5,q6. For the last transformation matrix,T6_G joint angle is z
ero.

### Homogeneous matrix from gripper pose

Rotation matrix from base link to gripper link is obtained by using tf API `tf.transformations.euler_matrix`. Given Roll,Pitch,Yaw of a body, its orientation can be obtained by rotation along x,y and z axis with resct to fixed world frame. Hence axes='sxyz' is used in `tf.transformations.euler_matrix` api.

Translation matrix is obtined by x,y,z values from gripper position. Assuming base link is placed at origin 0,0,0.

Homogeneous matrix between base link and gripper link can be obtained by concatenating ROtation matrix and Translation matrix. It is done according to below code.

```python
from sympy import Matrix
import tf

roll,pitch,yaw 	= 2.0,1.0,1.0
x,y,z 			= 1.2,1.5,2.0
R0_G = tf.transformations.euler_matrix(roll, pitch, yaw, axes='sxyz')
R0_G = Matrix(R0_G[0:3,0:3]) #extracting rotation matrix
t0_G = Matrix([[x],[y],[z]]) #transation matrix
H0_G = R0_G.row_join(t0_G) #combining rotational and translational
H0_G = H0_G.col_join(Matrix([[0,0,0,1]])) #last row to get homogeneous matrix

```

## Inverse Kinematics

### Calculating Wrist Position.

From the gripper pose, we know the position of Gripper in the World base frame. Let us call the vector V(0G)

Position of the wrist center in the world base frame = V(0W)

Position of the Gripper Position wrt Wrist Center in world base frame  = V(WG)

From triangulation,

V(0W) + V(WG) = V(0G) <br/>
V(0W) = V(0G) - V(WG) <br/>
We know V(0G) from gripper pose position. V(0G) = [[x],[y],[z]] <br/>
V(WG) in world frame can be defined in terms of rotation matrix from base link to wrist position and postion of gripper wrt wrist center in wrist center frame. <br/>
V(WG) = R(0_6) * t(6_G) <br/>
t(6_G) = Translation form frame 6 to gripper frame = [[d7],[0],[0]] <br/>
R0_6 = 3x3 matrix = [[a1,b1,c1],[d1,e1,f1],[g1,h1,i1]] <br/>
t(6_G) = 3x1 matrix <br/>
V(WG) = 3x1 matrix <br/>

V(0W) = V(0G) - V(WG) <br/>
V(0W)  = [[x - a1 * d7],[y - d1 * d7],[z - g1 * d7]] <br/>

![alt text][wc]
```python

def get_wrist_center(x,y,z,roll,pitch,yaw): #gamma - roll beta -yaw alpha - pitch
    
    R0_G = tf.transformations.euler_matrix(roll,pitch,yaw, axes='sxyz')
    a1,d1,g1 =  R0_G[0][0],R0_G[1][0],R0_G[2][0]
    d7 = s[d7]
    WC_x =   x - (d7 * a1 * 1.0) #in the world coordinates it is changing along x-direction..DH conventions Z-direction
    WC_y =   y - (d7 * d1 * 1.0)
    WC_z =   z - (d7 * g1 * 1.0)
    return WC_x,WC_y,WC_z

```
 

### calculating joint angles that determine postion of wrist center (theta1,theta2,theta3)

##### Theta1
From the above function, we can calculate wrist center once gripper position and orientation is known. <br/>
After wrist center is known, by projecting wrist center on to the XY-plane theta 1 is calculated. <br/>

theta1 = atan2(WC_y,WC_x)<br/>

Once theta1 is known, it is easy to calculate frame2 origin using rotation formulas.<br/>
Joint is rotated about Z-axis. Hence Z-coordinate remains constant.<br/><br/>
[[X,new],			[[cos(theta1) ,-sin(theta1)],  * [[x_old]  <br/>
 [Y_new]] = 		 [sin(theta1),cos(theta1)]]       [y_old]] <br/>

 x_old = 0.35(offset arm length), y_old =0  <br/>
					  
In this way as shown in the below code x2,y2 and z2 are calculate. <br/>

##### Theta3

For link3, there is already as offset angle because of s[a3] being non zero value. Calculate this as theta3_existing. <br/>
After rotation at prismatic joint 3, theta3_new can be calculated using cosine rule. It is easily depicted in picture below. <br/>
![alt text][theta3_existing]

![alt text][theta3_new]
FInally, once theta3_existing and theta3_new are calculated <br/>

joint_angle_3 = theta3 = theta3_new - theta3_existing <br/>

##### Theta2

Zero configuration of arm2 is at vertical position. <br/>
In case of elbow down configuration where arm3 is facing down relative to arm 2, <br/>
Theta2 = (pi/2) - (alpha + beta). Refer below image. <br/>

![alt text][theta2]

Here elbow up configuration is not considered. In case of elbow up configuration <br/>
theta2 = (pi/2) - (alpha - beta) <br/>

```python
def calculate_position_thetas(WC_x,WC_y,WC_z,alpha,beta,gamma):

    theta1 = atan2(WC_y,WC_x)
    X_new = 0.35 * cos(theta1)
    Y_new = 0.35 * sin(theta1)
    x2,y2,z2 = X_new,Y_new,s[d1]

    L2_3 = s[a2]
    L2_WC = sqrt((WC_x - x2) ** 2 + (WC_y - y2) ** 2 + (WC_z - z2) ** 2)
    L3_WC = sqrt(s[a3] ** 2 + s[d4] ** 2)

    theta3_existing = np.pi/2 - atan2(s[a3],s[d4])
    d3 = (L2_WC ** 2 - L3_WC ** 2 - L2_3 ** 2) / (2*L3_WC*L2_3)
    theta3_new = atan2(sqrt(1-d3**2),d3)
    theta3 = theta3_new - theta3_existing

    xy_proj = sqrt((WC_y-y2) ** 2 +  (WC_x - x2) ** 2)
    theta2_plane = atan2(WC_z- z2 ,xy_proj)
    d2 = ( L2_3 ** 2 + L2_WC ** 2 - L3_WC ** 2) / (2*L2_WC*L2_3)
    theta2_link = atan2(sqrt(1-d2**2),d2)
    theta2_vertical = theta2_plane + theta2_link
    theta2 = (np.pi/2) - (theta2_plane + theta2_link)
    return theta1,theta2,theta3
```



### calculating joint anlges that determine wrist orientation (theta4,theta5,theta6)

Once theta1,theta2 and theta3 are derived with above function `calculate_position_thetas` Rotation matrix from frame 0 to 3 in DH convention can be calculated.<br/>

T0_3 = T0_1 * T0_2 * T0_3 <br/>
R0_3 = T0_3[0:3,0:3]...Here substitute theta1,theta2 and theta3 values in q1,q2 and q3 respectively.<br/>

With roll,pitch,yaw of gripper being known, rotation matrix from base link to gripper link can be calculated.
`tf.transformations.euler_matrix(roll, pitch, yaw, axes='sxyz')` 

To rotate axis from world coordinates to link_6 coordinates, rotare about z-axis by -np.pi (R_zr) and then around y axis by 90 degrees(R_yr). <br/>
This rotations are intrinsic rotations, hence <br/>
r_corr_reverse = R_yr * R_zr <br/>

Note: `r_corr_reverse` is inverse of `R_corr` used during foreward kinematics to convert transformation matrix from DH axes to world axes <br/>

Therefore `R0_6_dh_value = R0_6_world_value * r_corr_reverse`

R0_3_dh and R0_6_dh is known, hence R3_6_dh_value is calculated using below equation <br/>
`R3_6_dh_value = Transpose(R0_3_dh_value) * R0_6_dh_value`
With R3_6 value known we can derive theta4,theta5 and theta6 either by uusing trignmometry approach where values in R3_6 are compared against symbols of T3_6 matrix. Other simple approach is to use tf API function `tf.transformations.euler_from_matrix`

But to use `tf.transformations.euler_from_matrix`, start and end coordinate frames should be in same directions.
From our DH axis assignment diagram, we can see that by rotating frame3 by 90 degrees about x-axis, frame3 and frame6 axis point in same directions.
Hence R3_6 is tf adjusted first and then theta4,theta5 and theta6 are calculated <br/>

`R3_6_tf_adjusted = R3_6_dh_value * R_x` <br/>
`theta4,theta5,theta6 = tf.transformations.euler_from_matrix(np.array(R3_6_tf_adjusted).astype(np.float64), "ryzy")` <br/>
As rotations are intrinsic and alog the axis y(at joint4),axis z (at joint 5) and axis y (at joint 6), axis='ryzy' argument is used.


```python

R_zr = Matrix([[cos(-np.pi),-sin(-np.pi),0],[sin(-np.pi),cos(-np.pi),0],[0,0,1]])
R_yr = Matrix([[cos(np.pi/2),0,sin(np.pi/2)],[0,1,0],[-sin(np.pi/2),0,cos(np.pi/2)]])
r_corr_reverse = R_yr * R_zr

def calculate_orientation_thetas(theta1,theta2,theta3,roll,pitch,yaw):

	T0_3 = T0_1 * T1_2 * T2_3
	R0_3 = T0_3[0:3,0:3]
	R0_3_dh_value = R0_3.evalf(subs={q1:theta1,q2:theta2,q3:theta3})
	T0_6_world_value = tf.transformations.euler_matrix(roll, pitch, yaw, axes='sxyz')
	R0_6_world_value = T0_6_world_value[0:3,0:3]
	R0_6_dh_value = R0_6_world_value * r_corr_reverse
	R3_6_dh_value = Transpose(R0_3_dh_value) * R0_6_dh_value
	R3_6_tf_adjusted = R3_6_dh_value * R_x
	theta4,theta5,theta6 = tf.transformations.euler_from_matrix(np.array(R3_6_tf_adjusted).astype(np.float64), "ryzy")
	return theta4,theta5,theta6
```


### IKServer.py

Inverse kinematics can be divided into three steps in total and in each step above explained function are used directly.

* Calculate wrist center from gripper position and pose. `get_wrist_center(x,y,z,roll,pitch,yaw)`
* Calculate joint angles that determine position of wrist center. `calculate_position_thetas(WC_x,WC_y,WC_z,alpha,beta,gamma)`
* Calculate joint angles that determine orientation of gripper. `calculate_orientation_thetas(theta1,theta2,theta3,roll,pitch,yaw)`

Same functions are used in the function `handle_calculate_IK` function when `calculate_ik` service is invoked. Same can be seen in below python code.

```python
def handle_calculate_IK(req):
	rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
	if len(req.poses) < 1:
		print "No valid poses received"
		return -1
	else:
		# Initialize service response
		joint_trajectory_list = []
		for x in xrange(0, len(req.poses)):
			# IK code starts here
			joint_trajectory_point = JointTrajectoryPoint()

			# Define DH param symbol
			T0_1 = get_homogeneous_matrix(a0,alpha0,d1,q1)
			T1_2 = get_homogeneous_matrix(a1,alpha1,d2,q2)
			T2_3 = get_homogeneous_matrix(a2,alpha2,d3,q3)
			T3_4 = get_homogeneous_matrix(a3,alpha3,d4,q4)
			T4_5 = get_homogeneous_matrix(a4,alpha4,d5,q5)
			T5_6 = get_homogeneous_matrix(a5,alpha5,d6,q6)

			T0_1 = T0_1.subs(s)
			T1_2 = T1_2.subs(s)
			T2_3 = T2_3.subs(s)
			T3_4 = T3_4.subs(s)
			T4_5 = T4_5.subs(s)
			T5_6 = T5_6.subs(s)

			T0_3 = T0_1 * T1_2 * T2_3

			px = req.poses[x].position.x
			py = req.poses[x].position.y
			pz = req.poses[x].position.z

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
				[req.poses[x].orientation.x, req.poses[x].orientation.y,
					req.poses[x].orientation.z, req.poses[x].orientation.w])
	 
			# Calculate joint angles using Geometric IK method
			alpha,beta,gamma = yaw,pitch,roll
			WC_x,WC_y,WC_z 	= get_wrist_center(px,py,pz,roll,pitch,yaw)
			theta1, theta2, theta3 = calculate_position_thetas(WC_x,WC_y,WC_z,alpha,beta,gamma)
			theta4, theta5, theta6 = calculate_orientation_thetas(theta1, theta2, theta3,roll,pitch,yaw,T0_3)

			# Populate response for the IK request
			# In the next line replace theta1,theta2...,theta6 by your joint angle variables
		joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
		joint_trajectory_list.append(joint_trajectory_point)

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		return CalculateIKResponse(joint_trajectory_list)
```
### Incomplete Work and Improvements

As discussed part of lecture series, to reach a wrist ceneter multiple combinations of joint angles is possible. Here only Elbow down configuration with first prismatic joint rotating only in first quadrant is explored. Other combinations are not calculated and joint angles are not checked against physical limits specified in urdf file.