#Project Drone_3DEstimator
This repository is estalished for the projet 4 of Udacity "Flying car" nanodegree program

---

## C++ Environment Setup

---
1. Clone the repository
git clone [project repository]（https://github.com/udacity/FCND-Estimation-CPP.git）
2. Import the code into your IDE, including open .sln, then retarget the SDK to the lastest version 
3. You should now be able to compile and run the estimation simulator just as you did in the controls project

You should now be able to compile and run the estimation simulator just as you did in the controls project
---

## Code Implementation
All the codes are included in the "QuadEstimatorEKF.cpp" file.
### 1. Standard deviation of measurement noise
The simulator is implemented, and the measurements from GPS and accelerometer are recorded into Graph1 and Graph2 txt files, then a scripts is used to load the recording and the method mean and stddev in numbepy class are adopted over the second column of file to get standard deviation.
### 2. Non-linear rate gyro integration
The code corresponding to modified rate gyro integration is from line 98 to line 119.
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
From the code snippet above, the rotation matrix is based on current euler angles to improve performance, the yaw angle is also normalized in the range from -pi to pi.
### 3. Prediction step
The code corresponding to roll pitch control is from 157 to 173.\
'''\
  predictedState(0) = curState(0) + predictedState(3) * dt;\
  predictedState(1) = curState(1) + predictedState(4) * dt;\
  predictedState(2) = curState(2) + predictedState(5) * dt;\
  predictedState(5) = curState(5) - CONST_GRAVITY * dt;

  V3F rotated_accel = attitude.Rotate_BtoI(accel);

  predictedState(3) = predictedState(3) + rotated_accel.x * dt;\
  predictedState(4) = predictedState(4) + rotated_accel.y * dt;\
  predictedState(5) = predictedState(5) + rotated_accel.z * dt;\
'''\
The code above is the new state update part, the position and velocity are updated accordingly.\
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
This code snippet in cpp file is from 180 to 189, corresponding to the gprime matrix to convert the body rate in local frame to global frame.
'''\
  gPrime(0, 3) = dt;\
  gPrime(1, 4) = dt;\
  gPrime(2, 5) = dt;\
  gPrime(3, 6) = (RbgPrime(0) * accel).sum() * dt;\
  gPrime(4, 6) = (RbgPrime(1) * accel).sum() * dt;\
  gPrime(5, 6) = (RbgPrime(2) * accel).sum() * dt;

  ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;\
'''\
The snippet in source file is from 269 to 276, the ekf covariance is updated via gprime function, acceleration is also accounted for as a command to calculate the gPrime matrix.
### 4. Magnotometer Update
The magnetometer update is from 333 to line 339.
'''
  hPrime(0, 6) = 1;\
  zFromX(0) = ekfState(6);\
  float diff = z(0) - zFromX(0);\
  if (diff > F_PI)\
	  zFromX(0) += 2.f*F_PI;\
  else if (diff < -F_PI) \
	  zFromX(0) -= 2.f*F_PI;\
'''
This update will include the magnetometer measurements, the error between magnetometer and the current state estimate is taken into consideration to correct the angle error.
### 5. GPS update
The counterpart is from line 300 to 312.
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
The hPrime for observation update is very simple with only constant values, the estimated measurements are current states accordingly except for the yaw anlge.
## Results
The final results for different scenarios are presented below.
1. Attitude scenario\
![scenario 6](/img/scenario-6-noise.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Scenario 6. Noise capturing of GPS and accelerometer<br />
The calculated deviation parameters capture the 68% measurements correcty.\
2. Attitude estimator\
![scenario 7](/img/scenario-7-attitude.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Scenario 7. Attitude estimator <br />
After the improvement bt non-linear update over gyro rate, the integration scheme results in an estimator of less than 0.1 rad/s of each euler angles  for duration more than 3 seconds.\
3. Predict state update\
![scenario 8](/img/scenario-8-predict.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Scenario 8. Position and velocity estimate<br />
The estimator can tranck the position and velocity correctly.\
4. Covariance update\
![scenario 9](/img/scenario-9-covariance.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Scenario 9.Covariance update<br />
The standard deviation of x position and corresponding velocity can grow correctly with the data within 1 second duration, the drift grow slowly over the duration., two sigma white lines bound the estimated state  properly.\
5. Predict state update\
![scenario 10](/img/scenario-10-mag.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Scenario 10. Magnetometer update<br />
The estimated standard deviation captures the error accurately and maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation.\
6. Predict state update\
![scenario 11](/img/scenario-11.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Scenario 11. Closed loop with GPS<br />
The entire simulation sycle is completed with the estimated position error less than 1 meter. Here the GPS in config files are increased somewhat to model the error accurately.\
7. Closed loop with custom controller\
![scenario 12](/img/scenario-12.JPG)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Scenario 12. Closed loop with custom controller<br />
The controller that has been integrated originally is replaced with the 3D controller from project 3. The parameter in config file are also de-tuned again, most of the parameters like kp for positiona and velocity control are all recudes somewhat, in particulat the body rate, about each axis is reduced a lot, the kp for yaw rate is limited to 15, otherwise the bigger value beyond this threshold will reult in the overshooting in z position in the end od the simulation cycle, no the two times' tranversing flying has misaligned trajectory which still close to each other. The drone at the end position of the simulation will drop a little lower that the target z position, the integrator ki parameter is increased to reduce this drift\
