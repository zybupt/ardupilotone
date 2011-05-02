/*
 * arducopter.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef ARDUCOPTER_H_
#define ARDUCOPTER_H_

//motor parameters
const float MOTOR_MAX = 1;
const float MOTOR_MIN = 0.1;

// position control loop
const float PID_POS_INTERVAL = 1/5; // 5 hz
const float PID_POS_P = 0.02;
const float PID_POS_I = 0;
const float PID_POS_D = 0.1;
const float PID_POS_LIM = 0.1; // about 5 deg
const float PID_POS_AWU = 0.0; // about 5 deg
const float PID_POS_Z_P = 0.5;
const float PID_POS_Z_I = 0.2;
const float PID_POS_Z_D = 0.5;
const float PID_POS_Z_LIM = 0.5;
const float PID_POS_Z_AWU = 0.1;

// attitude control loop
const float PID_ATT_INTERVAL  = 1/20; // 20 hz
const float PID_ATT_P = 0.03; // 0.1
const float PID_ATT_I = 0.03; // 0.0
const float PID_ATT_D = 0.03; // 0.1
const float PID_ATT_LIM = 0.1; // 0.01 // 10 % #define MOTORs
const float PID_ATT_AWU = 0.1; // 0.0
const float PID_YAWPOS_P = 1;
const float PID_YAWPOS_I = 0.1;
const float PID_YAWPOS_D = 0;
const float PID_YAWPOS_LIM = 1; // 1 rad/s
const float PID_YAWPOS_AWU = 1; // 1 rad/s
const float PID_YAWSPEED_P = 1;
const float PID_YAWSPEED_I = 0;
const float PID_YAWSPEED_D = 0.5;
const float PID_YAWSPEED_LIM = 0.1; // 0.01 // 10 % MOTORs
const float PID_YAWSPEED_AWU = 0.0;
const float PID_YAWSPEED_DFCUT = 30.0; // 30 Radians, about 10 Hz

// mixing
const float MIX_REMOTE_WEIGHT = 1;
const float MIX_POSITION_WEIGHT = 1;
const float MIX_POSITION_Z_WEIGHT = 1;
const float MIX_POSITION_YAW_WEIGHT = 1;

const float THRUST_HOVER_OFFSET = 0.475;

#endif /* ARDUCOPTER_H_ */
