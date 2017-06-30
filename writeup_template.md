## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/kuka_schematic.png
[image2]: ./misc_images/wc.png
[image3]: ./misc_images/theta1.gif
[image4]: ./misc_images/3d_scheme.jpg
[image5]: ./misc_images/cosine_law.png


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Using scheme *Image 1* from lesson about KR210 Forward Kinematic and URDF file `kr210.urdf.xacro` I've made analysis and filled DH parameters table *Table 1*

![alt text][image1]
*Image 1 - Kuka KR210 Sketch in zero configuration*

![alt text][image4]
*Image 2 - 3D figure for better understanding where each angle is*

 i | alpha(i-1) | a(i-1) | d(i-1) | q(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1 
2 | -pi/2 | 0.35 | 0 | q2-pi/2
3 | 0 | 1.25 | 0 | q3 
4 |  -pi/2 | -0.054 | 1.50 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
7 | 0 | 0 | 0.303 | 0

*Table 2 - DH parameters table*

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I created script `forward_kinematics.py` for forward kinematics. Here is individual transformation matrices:

```python
# Individual transformation matrices
T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
T4_5 = T4_5.subs(s) 

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
T5_6 = T5_6.subs(s) 

T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
T6_G = T6_G.subs(s)


# Generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
R_roll  = Matrix([[           1,          0,          0],
                  [           0,  cos(roll), -sin(roll)],
                  [           0,  sin(roll),  cos(roll)]])

R_pitch = Matrix([[  cos(pitch),          0,     sin(p)],
                  [           0,          1,          0],
                  [ -sin(pitch),          0,     cos(p)]])

R_yaw   = Matrix([[    cos(yaw),  -sin(yaw),          0],
                  [    sin(yaw),   cos(yaw),          0],
                  [           0,          0,          1]])

R0_6 = R_roll * R_pitch * R_yaw
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

It was a very difficult task. Part of the solutions was found during the discussion on the Slack channel, as well as in the third-party videos with kinematics lessons on YouTube.

1. Prepared DH parameter table for the arm.

2. Found wrist center using formula

![alt text][image2]

3. Found Theta 1 using arctangent from two sides of right triangle

![alt text][image3]

4. Found Theta 3 using Law of cosine. The solution was found in the discussion on Slack channel.

![alt text][image5]

5. Found Theta 2 using Theta 3

6. Found Theta 4, 5 and 6 using Euler angles `tf.transformations.euler_from_matrix`

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The equations which are used in `IK_server.py` described above.
I had to move all matrices initializations out of the main loop. Also I did not use `simplify` function which significantly increased productivity.

