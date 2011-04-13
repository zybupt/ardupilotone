/*
 * mikrokopter.h
 *
 *  Created on: Apr 6, 2011
 *      Author: nkgentry
 */

#ifndef MIKROKOPTER_H_
#define MIKROKOPTER_H_

#define VEHICLE_MIKROKOPTER

//motor parameters
#define MOTOR_MAX  1
#define MOTOR_MIN  0.1

// position control loop
#define PID_POS_INTERVAL  1/5 // 5 hz
#define PID_POS_P 0.02
#define PID_POS_I 0
#define PID_POS_D 0.1
#define PID_POS_LIM 0.1 // about 5 deg
#define PID_POS_AWU 0.0 // about 5 deg
#define PID_POS_Z_P 0.5
#define PID_POS_Z_I 0.2
#define PID_POS_Z_D 0.5
#define PID_POS_Z_LIM 0.5
#define PID_POS_Z_AWU 0.1
#define VEL_OFFSET_X 0.0
#define VEL_OFFSET_Y 0.0

// attitude control loop
#define PID_ATT_INTERVAL  1/20 // 20 hz
#define PID_ATT_P 0.1
#define PID_ATT_I 0.0
#define PID_ATT_D 0.1
#define PID_ATT_LIM 0.01 // 10 % #define MOTORs
#define PID_ATT_AWU 0.0
#define PID_YAWPOS_P 1
#define PID_YAWPOS_I 0.1
#define PID_YAWPOS_D 0
#define PID_YAWPOS_LIM 1 // 1 rad/s
#define PID_YAWPOS_AWU 1 // 1 rad/s
#define PID_YAWSPEED_P 1
#define PID_YAWSPEED_I 0
#define PID_YAWSPEED_D 0.5
#define PID_YAWSPEED_LIM 0.01 // 10 % #define MOTORs
#define PID_YAWSPEED_AWU 0.0
#define ATT_OFFSET_X 0.0
#define ATT_OFFSET_Y 0.0
#define ATT_OFFSET_Z 0.0

// mixing
#define MIX_REMOTE_WEIGHT  1
#define MIX_POSITION_WEIGHT 1
#define MIX_POSITION_Z_WEIGHT 1
#define MIX_POSITION_YAW_WEIGHT  1
#define MIX_OFFSET_WEIGHT 0

// waypoint
#define POSITION_SETPOINT_X  0
#define POSITION_SETPOINT_Y  0
#define POSITION_SETPOINT_Z  0
#define POSITION_SETPOINT_YAW  0.0

#define THRUST_HOVER_OFFSET  0.475

#endif /* MIKROKOPTER_H_ */
