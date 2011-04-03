#ifndef defaultControllers_H
#define defaultControllers_H

#include "ArduPilotOne.h"
#include "AP_Controller.h"
#include "AP_Var.h"
#include <avr/pgmspace.h>
#include "defines.h"

#if VEHICLE_TYPE == CAR

class CarController: public AP_Controller {
private:
	// state
	const float * velocity;
	const float * heading;

	// control variables
	const float * headingCommand;
	const float * velocityCommand;

	// channels
	AP_RcChannel * modeCh;
	AP_RcChannel * steeringCh;
	AP_RcChannel * throttleCh;

	// control mode
	AP_Var_group _group;AP_Uint8 _mode;

public:
	CarController(AP_Var::Key cntrlKey, AP_Var::Key pidStrKey,
			AP_Var::Key pidThrKey, const float * heading,
			const float * velocity, const float * headingCommand,
			const float * velocityCommand, AP_RcChannel * modeCh,
			AP_RcChannel * steeringCh, AP_RcChannel * throttleCh) :
	heading(heading), velocity(velocity), headingCommand(headingCommand),
	velocityCommand(velocityCommand), modeCh(modeCh),
	steeringCh(steeringCh), throttleCh(throttleCh),
	_group(cntrlKey, PSTR("CNTRL_")),
	_mode(&_group, 1, 0, PSTR("MODE")) {
		Serial.println_P(PSTR("initializing car controller"));

		// steering control loop
		addBlock(new SumGain(headingCommand, &one, heading, &negativeOne));
		addBlock(new Pid(pidStrKey, PSTR("STR_"), 1, 1, 1, 1, 20));
		addBlock(new ToServo(steeringCh));

		// throttle control loop
		addBlock(new SumGain(velocityCommand, &one, velocity, &negativeOne));
		addBlock(new Pid(pidThrKey, PSTR("THR_"), 1, 1, 1, 1, 20));
		addBlock(new ToServo(throttleCh));
	}
	virtual void update(const double dt) {
		// read mode switch
		modeCh->readRadio();

		// manual
		if (_mode == 0) {
			steeringCh->readRadio();
			throttleCh->readRadio();
			//Serial.println("manual");

		} else { // auto
			AP_Controller::update(dt);
			//Serial.println("automode");
		}

		//Serial.printf("steering pwm :\t");
		//Serial.printf("%7d\t",steeringCh->getPwm());
		//Serial.println();

		//Serial.printf("throttle pwm :\t");
		//Serial.printf("%7d\t",throttleCh->getPwm());
		//Serial.println();

		//delay(100);
	}

};

void controllerInit() {
	_rc.push_back(new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 45));
	_rc.push_back(new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0, 45));
	_rc.push_back(new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1, 100));

	_controller = new CarController(k_cntrl, k_pidStr, k_pidThr,
			&(navigator()->yaw), &(navigator()->groundSpeed),
			&guide()->headingCommand, &guide()->groundSpeedCommand, _rc[0],
			_rc[1], _rc[2]);

}

#elif VEHICLE_TYPE == QUAD

class QuadController: public AP_Controller {
private:
	//	// state
	//	AP_Float roll;
	//	AP_Float airspeed;
	//	AP_Float velocity;
	//	AP_Float heading;
	//
	//	// servo positions
	//	AP_Float steering;
	//	AP_Float throttle;
	//
	//	// control variables
	//	AP_Float headingCommand;
	//	AP_Float airspeedCommand;
	//	AP_Float rollCommand;
	//
	// position variables
	float commandNorth;
	float commandEast;
	float commandDown;
	float commandYaw;
#define PID_POS_P 0.5
#define PID_POS_I 0.0
#define PID_POS_D 1.0
#define PID_POS_AWU 0
#define PID_POS_LIM 1
#define PID_POS_Z_P 0.5
#define PID_POS_Z_I 0.0
#define PID_POS_Z_D 1.0
#define PID_POS_Z_AMU 0
#define PID_POS_Z_LIM 1

	// bridge variables
	float commandRoll;
	float commandPitch;

	// attitude variables
#define PID_ATT_P 0.5
#define PID_ATT_I 0.0
#define PID_ATT_D 1.0
#define PID_ATT_AWU 0
#define PID_YAWPOS_P 0.5
#define PID_YAWPOS_I 0.0
#define PID_YAWPOS_D 1.0
#define PID_YAWPOS_AWU 0
#define PID_YAWSPEED_P 0.5
#define PID_YAWSPEED_I 0.0
#define PID_YAWSPEED_D 1.0
#define PID_YAWSPEED_AWU 0

	// mix manual variables
#define MIX_REMOTE_WEIGHT 0
#define MIX_OFFSET_WEIGHT 0
#define ATT_OFFSET_X 0
#define ATT_OFFSET_Y 0
#define ATT_OFFSET_Z 0

	// QUAD controller output
#define THRUST_HOVER_OFFSET 0

	// mix variables
	float thrustMix;
	float rollMix;
	float yawMix;
	float pitchMix;

	// motor mix saturation
#define motorMin 10
#define motorMax 255

	// motor channels
	AP_RcChannel * chMode;
	AP_RcChannel * chLeft;
	AP_RcChannel * chRight;
	AP_RcChannel * chFront;
	AP_RcChannel * chBack;

public:
	QuadController(AP_Var::Key cntrlKey, AP_Var::Key chLeftKey,
			AP_Var::Key chRightKey, AP_Var::Key chFrontKey,
			AP_Var::Key chBackKey, AP_Var::Key pidRollKey,
			AP_Var::Key pidPitchKey, AP_Var::Key pidYawKey,
			const float * velocity, const float * velocityCommand,
			const float * heading, const float * headingCommand,
			AP_RcChannel * chMode, AP_RcChannel * chLeft,
			AP_RcChannel * chRight, AP_RcChannel * chFront,
			AP_RcChannel * chBack) :
		chLeft(chLeft), chRight(chRight), chFront(chFront),
		chBack(chBack)
	{
		Serial.println_P(PSTR("initializing quad controller"));
		//addBlock(new QuadMix(getRc(chRoll, chPitch, chYaw, chThr)));
		// RC channels

		//addBlock(new Saturate(motorMin,motorMax));
		//addBlock(new ToServo(chLeft));

		////
		// MOTOR MIX												//
		//													//
		// inputs:  rollMix, pitchMix, yawMix, thrustMix						 	//
		// outputs: (to servo: chLeft, chRight, chFront, chBack)						//
		//													//
		////
		/*
		 addBlock(new QuadMix(getRc(chRoll, chPitch, chYaw, chThr)));
		 */
		// Left Motor
		addBlock(new SumGain(&thrustMix, &one, &rollMix, &one, &yawMix, &one));
		/*			addBlock(new Saturate(motorMin,motorMax);
		 addBlock(new ToServo(chLeft);

		 // Right Motor
		 addBlock(new SumGain(thrustMix,one,rollMix,negone,yawMix,one));
		 addBlock(new Saturate(motorMin,motorMax);
		 addBlock(new ToServo(chRight);

		 // Front Motor
		 addBlock(new SumGain(thrustMix,one,pitchMix,one,yawMix,negone));
		 addBlock(new Saturate(motorMin,motorMax);
		 addBlock(new ToServo(chFront);

		 // Back Motor
		 addBlock(new SumGain(thrustMix,one,pitchMix,negone,yawMix,negone));
		 addBlock(new Saturate(motorMin,motorMax);
		 addBlock(new ToServo(chBack);
		 */
		 }

};

	void controllerInit() {
		_rc.push_back(new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 45));
		_rc.push_back(new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 0, 1));
		_rc.push_back(new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 1, 1));
		_rc.push_back(new AP_RcChannel(k_chFront, PSTR("FRONT_"), APM_RC, 2, 1));
		_rc.push_back(new AP_RcChannel(k_chBack, PSTR("BACK_"), APM_RC, 3, 1));

		_controller = new QuadController(k_cntrl, k_chLeft, k_chRight,
				k_chFront, k_chBack, k_pidRoll, k_pidPitch, k_pidYaw,
				&navigator()->groundSpeed, &guide()->groundSpeedCommand,
				&navigator()->yaw, &guide()->headingCommand, _rc[0],
				_rc[1], _rc[2], _rc[3], _rc[4]);
	}

#elif VEHICLE_TYPE == PLANE

	class PlaneController : public AP_Controller
	{
	private:
		// state
		AP_Float roll;
		AP_Float airspeed;
		AP_Float velocity;
		AP_Float heading;

		// servo positions
		AP_Float steering;
		AP_Float throttle;

		// control variables
		AP_Float headingCommand;
		AP_Float airspeedCommand;
		AP_Float rollCommand;

		// channels
		static const uint8_t chRoll = 0;
		static const uint8_t chPitch = 1;
		static const uint8_t chYaw = 2;

	public:
		PlaneController(AP_Var::Key chRollKey, AP_Var::Key chPitchKey, AP_Var::Key chYawKey,
				AP_Var::Key pidRollKey, AP_Var::Key pidPitchKey, AP_Var::Key pidYawKey) : {
			// rc channels
			addCh(new AP_RcChannel(chRollKey,PSTR("ROLL"),APM_RC,chRoll,45));
			addCh(new AP_RcChannel(chPitchKey,PSTR("PTCH"),APM_RC,chPitch,45));
			addCh(new AP_RcChannel(chYawKey,PSTR("YAW"),APM_RC,chYaw,45));

			// pitch control loop
#if AIRSPEED_SENSOR == ENABLED
			// pitch control loop w/ airspeed
			addBlock(new SumGain(airspeedCommand,AP_Float_unity,airspeed,AP_Float_negative_unity));
#else
			// cross feed variables
			addBlock(new SumGain(roll,kffPitchCompk,throttleServo,kffT2P));
#endif
			addBlock(new Pid(pidPitchKey,PSTR("PTCH"),0.1,0,0,1,20));
			addBlock(new ToServo(getRc(chPitch)));

			// roll control loop
			addBlock(new SumGain(headingCommand,one,heading,negOne));
			addBlock(new Pid(headingkP,headingKI,headingKD));
			addBlock(new Sink(rollCommand));
			addBlock(new SumGain(rollCommand,one,roll,negOne));
			addBlock(new Pid(rollKP,rollKI,rollKD));
			addBlock(new ToServo(getRc(chRoll)));

			// throttle control loop
			addBlock(new SumGain(airspeedCommand,one,airspeed,negOne));
			addBlock(new Pid(throttleKP,throttleKI,throttleKD));
			addBlock(new ToServo(getRc(chThr)));
		}
	};

	void controllerInit()
	{
		controller = new planeController;
	}

#endif // VEHICLE_TYPE
#endif // defaultControllers_H
// vim:ts=4:sw=4:expandtab
