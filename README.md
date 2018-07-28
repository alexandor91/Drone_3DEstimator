#Project Drone_3DEstimator
This repository is estalished for the projet 4 of Udacity "Flying car" nanodegree program

---

## C++ Environment Setup

---
1. Download or clone the [project repository](https://github.com/udacity/FCND-Controls-CPP) onto the local pc.  
2. Use any C++ editor to fire the project, I used Visual Studio as editor, load the "sln" project and retarget the solution for new version SDK, then compile and run the program.
---

## Code Implementation
All the codes are included in the "QuadControl.cpp" file.
### 1. Bodyrate Controller
The code corresponding to body control is from line 119 to line 128.\
'''\
float p_error = pqrCmd[0] - pqr[0];\
float u_bar_p = Ixx * kpPQR[0] * p_error;

float q_error = pqrCmd[1] - pqr[1];\
float u_bar_q = Iyy * kpPQR[1] * q_error;

float r_error = pqrCmd[2] - pqr[2];\
float u_bar_r = Izz * kpPQR[2] * r_error;

V3F momentCmd(u_bar_p, u_bar_q, u_bar_r);\
'''\
Fron the code snippet above, it is obvious that the commanded moments are proportional to the difference of body rates, the intertia od the drone about each axis is also taken into account.
### 2. Roll Pitch Controller
The code corresponding to roll pitch control is from 157 to 173.\
'''\
 float c_d = collThrustCmd / mass;\
  if (collThrustCmd > 0.0){\
    float target_R13 = -CONSTRAIN(accelCmd[0] / c_d, -maxTiltAngle, maxTiltAngle);\
    float target_R23 = -CONSTRAIN(accelCmd[1] / c_d, -maxTiltAngle, maxTiltAngle);\
    pqrCmd[0] = (1 / R(2, 2)) * (-R(1, 0) * kpBank * (R(0, 2) - target_R13) + R(0, 0) * kpBank * (R(1, 2) - target_R23));\
    pqrCmd[1] = (1 / R(2, 2)) * (-R(1, 1) * kpBank * (R(0, 2) - target_R13) + R(0, 1) * kpBank * (R(1, 2) - target_R23));\
  }
  else {\
  pqrCmd[0] = 0.0;\
  pqrCmd[1] = 0.0;\
  collThrustCmd = 0.0;\
  }\
'''\
To calculate the target angle, drone mass is accounted for. The element in the non-linear rotation matrix is responsible for the non-linear transform from local acceleration to body rate. This pitch and roll controller is cascaded to the body rate controller, so the output is desired body rate.
### 3. Altitude Controller
The code of this controller part is from line 203 to line 211.\
'''\
float error = posZCmd - posZ;\
float h_dot_cmd = kpPosZ * error + velZCmd;\
h_dot_cmd = CONSTRAIN(h_dot_cmd, -maxAscentRate, maxDescentRate);\
integratedAltitudeError += error * dt;\
float acceleration_cmd = accelZCmd + kpVelZ * (h_dot_cmd - velZ) + KiPosZ * integratedAltitudeError;

float R33 = R(2,2);\
thrust = mass * (9.81f - acceleration_cmd) / R33;\
thrust = CONSTRAIN(thrust, 4 * minMotorThrust, 4 * maxMotorThrust);\
'''\
The bottom two lines convert the desired acceleration to collective thrust, also includes the non-linear effects from non-zero roll or pitch angles, the two p controllers, cascaded together form the second order control system, one for velocity control, the other for position control. Aside from the normal p term, the integrator is adopted to handle the shift of weight presented in one of the drones of scenario 4. 
### 4. Lateral Position Controller
Line 247 to line 263 is the code for horizontal position control.\
'''\
velCmd.x = kpPosXY * (posCmd.x - pos.x);\
velCmd.y = kpPosXY * (posCmd.y - pos.y);\
float vel_norm = sqrt(velCmd.x * velCmd.x + velCmd.y * velCmd.y);
 
if (vel_norm > maxSpeedXY) {\
    velCmd.x = CONSTRAIN(velCmd.x, -velCmd.x * maxSpeedXY / vel_norm, velCmd.x * maxSpeedXY / vel_norm);\
    velCmd.y = CONSTRAIN(velCmd.y, -velCmd.y * maxSpeedXY / vel_norm, velCmd.y * maxSpeedXY / vel_norm);\
  }\
accelCmd.x += kpPosXY * (posCmd.x - pos.x) + kpVelXY * (velCmd.x - vel.x);\
accelCmd.y += kpPosXY * (posCmd.y - pos.y) + kpVelXY * (velCmd.y - vel.y);

float accel_norm = sqrt(accelCmd.x * accelCmd.x + accelCmd.y * accelCmd.y);

if (accel_norm > maxAccelXY) {\
	  accelCmd.x = CONSTRAIN(accelCmd.x, -accelCmd.x * maxAccelXY / accel_norm, accelCmd.x * maxAccelXY / accel_norm);\
	  accelCmd.y = CONSTRAIN(accelCmd.y, -accelCmd.y * maxAccelXY / accel_norm, accelCmd.y * maxAccelXY / accel_norm);\
  }\
'''\
The local NE position and velocity  are used to generate a commanded local acceleration, here the desired velocity and the desired accceleration are constrained to the bounding range to ensure safe operation in reality.
### 5. Yaw Controller
Line 284 to line 291 for yaw control.\
'''\
yawCmd = fmodf(yawCmd, 2.0 * 3.1416f);\
float yaw_error = yawCmd - yaw;\
if (yaw_error > 3.1416f)\
	  yaw_error = yaw_error - (2.0f * 3.1416f);\
else if (yaw_error < -3.1416f)\
	  yaw_error = yaw_error + (2.0f * 3.1416f);
  
yawRateCmd = kpYaw * yaw_error;\
'''\
The code snippet above is very simple, the controller output is proportional  to the difference between disired yaw and actual yaw.
### 6. Motor Commands
Line 73 to line 92 for four motor commands given the moment and collective thrust commands.\
'''\
float l = L / sqrt(2);\
float term_A = collThrustCmd;\
float term_B = momentCmd[0]/ l;\
float term_C = momentCmd[1] / l;\
float term_D = -momentCmd[2] /kappa;

float F0 = (term_A + term_B + term_C + term_D) / 4.0f; //front left\
float F1 = (term_A + term_C - term_B - term_D) / 4.0f; //front right\
float F2 = (term_A + term_B - term_C - term_D) / 4.0f; //rear left\
float F3 = (term_A + term_D - term_B - term_C) / 4.0f; //rear right

//printf("F0 force %f/n", term_A);\
//printf("F1 force %f/n", term_B);\
//printf("F2 force %f/n", term_C);\
//printf("F3 force %f/n", term_D);

cmd.desiredThrustsN[0] = CONSTRAIN(F0, minMotorThrust, maxMotorThrust);\
cmd.desiredThrustsN[1] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust);\
cmd.desiredThrustsN[2] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust);\
cmd.desiredThrustsN[3] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust);\
'''\
The output here is the individual thrust force generated by each motor, the three moments about respective axis along with the collective force are mapped accordingly, to note is the moment about z axis, in local NED all the upward thrust are negative.

## Results
The final results for different scenarios are presented below.
1. Attitude scenario\
![Attitude control](/img/attitude-scenario.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Attitude counteracting the yaw pertubation<br />
The drone can be resistant to the rotational rate about x, and recovered to level attitude shortly.\
2. Attitude scenario\
![Position control](/img/position-scenario.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Position control of two drones, one with yaw offset<br />
The drone can be controlled to move towards the target position, yaw angle can be oriented to zero shortly.
3. Non-idealities scenario\
![Attitude control](/img/non-ideality.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Non-idealities control<br />
Non idealiies includes the shifting mass and the actual heavy mass, Fron picture above, the three drones all reach the target position successfully, drone 3's trajectory is a smooth curve.
4. Trajectory scenario\
![Attitude control](/img/trajectory-scenario5.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Trajectory follower<br />
Here only drone 1 passes, the lighter drone 2 based on the same settings from the scenario 4 can perform the tracking roughly, it cannot hold to path fairly well, if the velocity from the trajectory can be generated as well, the tracking controller may perform better for drone 2. 
