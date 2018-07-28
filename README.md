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
  float phi = rollEst;\
  float theta = pitchEst;

  Mat3x3F rotation = Mat3x3F::Zeros();\
  rotation(0, 0) = 1;\
  rotation(0, 1) = sin(phi) * tan(theta);\
  rotation(0, 2) = cos(phi) * tan(theta);\
  rotation(1, 0) = 0;\
  rotation(1, 1) = cos(phi);\
  rotation(1, 2) = -sin(phi); \
  rotation(2, 0) = 0;\
  rotation(2, 1) = sin(phi) / cos(theta);\
  rotation(2, 2) = cos(phi) / cos(theta);
  
  V3F euler_dot = rotation * gyro;

  float predictedRoll = rollEst + dtIMU * euler_dot.x;\
  float predictedPitch = pitchEst + dtIMU * euler_dot.y;\
  ekfState(6) = ekfState(6) + dtIMU * euler_dot.z;\
  // normalize yaw to -pi .. pi\
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;\
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;\
'''\
Fron the code snippet above, it is obvious that the commanded moments are proportional to the difference of body rates, the intertia od the drone about each axis is also taken into account.
### 3. Roll Pitch Controller
The code corresponding to roll pitch control is from 157 to 173.\
'''\
  float phi = rollEst;\
  float theta = pitchEst;

  Mat3x3F rotation = Mat3x3F::Zeros();\
  rotation(0, 0) = 1;\
  rotation(0, 1) = sin(phi) * tan(theta);\
  rotation(0, 2) = cos(phi) * tan(theta);\
  rotation(1, 0) = 0;\
  rotation(1, 1) = cos(phi);\
  rotation(1, 2) = -sin(phi); \
  rotation(2, 0) = 0;\
  rotation(2, 1) = sin(phi) / cos(theta);\
  rotation(2, 2) = cos(phi) / cos(theta);
  
  V3F euler_dot = rotation * gyro;

  float predictedRoll = rollEst + dtIMU * euler_dot.x;\
  float predictedPitch = pitchEst + dtIMU * euler_dot.y;\
  ekfState(6) = ekfState(6) + dtIMU * euler_dot.z;\
  // normalize yaw
  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;\
  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;\
'''\
'''\
  float phi = roll;\
  float theta = pitch;\
  float psi = yaw;
  
  RbgPrime(0, 0) = - cos(theta) * sin(psi);\
  RbgPrime(0, 1) = - sin(phi) * sin(theta) * sin(psi) - cos(phi) * cos(psi);\
  RbgPrime(0, 2) = - cos(phi) * sin(theta) * sin(psi) + sin(phi) * cos(psi);\
  RbgPrime(1, 0) = cos(theta) * cos(psi);\
  RbgPrime(1, 1) = sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi);\
  RbgPrime(1, 2) = cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi);\
'''\
'''\
  gPrime(0, 3) = dt;\
  gPrime(1, 4) = dt;\
  gPrime(2, 5) = dt;\
  gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;\
  gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;\
  gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;

  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;\
'''\
### 4. Magnotometer Update
'''
  hPrime(0, 6) = 1;\
  zFromX(0) = ekfState(6);\
  float diff = z(0) - zFromX(0);\
  if (diff > F_PI)\
	  zFromX(0) += 2.f*F_PI;\
  else if (diff < -F_PI) \
	  zFromX(0) -= 2.f*F_PI;\
'''
### 5. GPS update
'''
  hPrime(0, 0) = 1;\
  hPrime(1, 1) = 1;\
  hPrime(2, 2) = 1;\
  hPrime(3, 3) = 1;\
  hPrime(4, 4) = 1;\
  hPrime(5, 5) = 1;

  zFromX(0) = ekfState(0);\
  zFromX(1) = ekfState(1);\
  zFromX(2) = ekfState(2);\
  zFromX(3) = ekfState(3);\
  zFromX(4) = ekfState(4);\
  zFromX(5) = ekfState(5);\
'''
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
