#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include <string>

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float l = L / sqrt(2);
  float term_A = collThrustCmd;
  float term_B = momentCmd[0]/ l;
  float term_C = momentCmd[1] / l;
  float term_D = -momentCmd[2] /kappa;

  float F0 = (term_A + term_B + term_C + term_D) / 4.0f; //front left
  float F1 = (term_A + term_C - term_B - term_D) / 4.0f; //front right
  float F2 = (term_A + term_B - term_C - term_D) / 4.0f; //rear left
  float F3 = (term_A + term_D - term_B - term_C) / 4.0f; //rear right

  //printf("F0 force %f/n", term_A);
  //printf("F1 force %f/n", term_B);
  //printf("F2 force %f/n", term_C);
  //printf("F3 force %f/n", term_D);

  cmd.desiredThrustsN[0] = CONSTRAIN(F0, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  //V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //V3F rate_error = pqrCmd - pqr;
  //V3F inertial(Ixx, Iyy, Izz);
  //momentCmd = inertial * (kpPQR * rate_error);
  float p_error = pqrCmd[0] - pqr[0];
  float u_bar_p = Ixx * kpPQR[0] * p_error;

  float q_error = pqrCmd[1] - pqr[1];
  float u_bar_q = Iyy * kpPQR[1] * q_error;

  float r_error = pqrCmd[2] - pqr[2];
  float u_bar_r = Izz * kpPQR[2] * r_error;

  V3F momentCmd(u_bar_p, u_bar_q, u_bar_r);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
 float c_d = collThrustCmd / mass;
  if (collThrustCmd > 0.0){
    float target_R13 = -CONSTRAIN(accelCmd[0] / c_d, -maxTiltAngle, maxTiltAngle);
    float target_R23 = -CONSTRAIN(accelCmd[1] / c_d, -maxTiltAngle, maxTiltAngle);

    pqrCmd[0] = (1 / R(2, 2)) * \
    (-R(1, 0) * kpBank * (R(0, 2) - target_R13) + \
    R(0, 0) * kpBank * (R(1, 2) - target_R23));
    pqrCmd[1] = (1 / R(2, 2)) * \
    (-R(1, 1) * kpBank * (R(0, 2) - target_R13) + \
    R(0, 1) * kpBank * (R(1, 2) - target_R23));
  }
  else {
  pqrCmd[0] = 0.0;
  pqrCmd[1] = 0.0;
  collThrustCmd = 0.0;
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float error = posZCmd - posZ;
  float h_dot_cmd = kpPosZ * error + velZCmd;
  h_dot_cmd = CONSTRAIN(h_dot_cmd, -maxAscentRate, maxDescentRate);
  integratedAltitudeError += error * dt;
  float acceleration_cmd = accelZCmd + kpVelZ * (h_dot_cmd - velZ) + KiPosZ * integratedAltitudeError;

  float R33 = R(2,2);
  thrust = mass * (9.81f - acceleration_cmd) / R33;
  thrust = CONSTRAIN(thrust, 4 * minMotorThrust, 4 * maxMotorThrust);
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   velCmd.x = kpPosXY * (posCmd.x - pos.x);
   velCmd.y = kpPosXY * (posCmd.y - pos.y);
  float vel_norm = sqrt(velCmd.x * velCmd.x + velCmd.y * velCmd.y);
 
  if (vel_norm > maxSpeedXY) {
	  velCmd.x = CONSTRAIN(velCmd.x, -velCmd.x * maxSpeedXY / vel_norm, velCmd.x * maxSpeedXY / vel_norm);
	  velCmd.y = CONSTRAIN(velCmd.y, -velCmd.y * maxSpeedXY / vel_norm, velCmd.y * maxSpeedXY / vel_norm);
  }
  accelCmd.x += kpPosXY * (posCmd.x - pos.x) + kpVelXY * (velCmd.x - vel.x);
  accelCmd.y += kpPosXY * (posCmd.y - pos.y) + kpVelXY * (velCmd.y - vel.y);

  float accel_norm = sqrt(accelCmd.x * accelCmd.x + accelCmd.y * accelCmd.y);

  if (accel_norm > maxAccelXY) {
	  accelCmd.x = CONSTRAIN(accelCmd.x, -accelCmd.x * maxAccelXY / accel_norm, accelCmd.x * maxAccelXY / accel_norm);
	  accelCmd.y = CONSTRAIN(accelCmd.y, -accelCmd.y * maxAccelXY / accel_norm, accelCmd.y * maxAccelXY / accel_norm);
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  yawCmd = fmodf(yawCmd, 2.0 * 3.1416f);
  float yaw_error = yawCmd - yaw;
  if (yaw_error > 3.1416f)
	  yaw_error = yaw_error - (2.0f * 3.1416f);
  else if (yaw_error < -3.1416f)
	  yaw_error = yaw_error + (2.0f * 3.1416f);
  
  yawRateCmd = kpYaw * yaw_error;
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
