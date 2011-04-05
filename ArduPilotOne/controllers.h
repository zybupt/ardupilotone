#ifndef defaultControllers_H
#define defaultControllers_H

//#include "ArduPilotOne.h"
#include "AP_Controller.h"
#include "AP_RcChannel.h"
#include "AP_Var.h"
#include <avr/pgmspace.h>

#include "AP_Navigator.h"

namespace apo {

class CarController: public AP_Controller {
private:
	// control mode
	AP_Var_group _group;AP_Uint8 _mode;
	static const uint8_t chMode = 0;
	static const uint8_t chStr = 1;
	static const uint8_t chThr = 2;
public:
	CarController(AP_Var::Key cntrlKey, AP_Var::Key pidStrKey,
			AP_Var::Key pidThrKey, AP_Navigator * nav,
			AP_Guide * guide, AP_HardwareAbstractionLayer * hal) :
		AP_Controller(nav, guide, hal), _group(cntrlKey, PSTR("CNTRL_")),
				_mode(&_group, 1, 0, PSTR("MODE")) {
		_hal->debug->println_P(PSTR("initializing car controller"));

		// NOTE, if you change this order, change the numbers above
		_hal->rc.push_back(new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 1));
		_hal->rc.push_back(new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0, 45));
		_hal->rc.push_back(new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1, 100));

		// steering control loop
		addBlock(new SumGain(&(_guide->headingCommand), &one,&(_nav->yaw), &negativeOne));
		addBlock(new Pid(pidStrKey, PSTR("STR_"), 1, 0, 0, 0, 20));
		addBlock(new ToServo(_hal->rc[chMode])); // index depends on order of channels pushed back into _hal->rc

		// throttle control loop
		addBlock(new SumGain(&(_guide->groundSpeedCommand), &one,&(_nav->groundSpeed), &negativeOne));
		addBlock(new Pid(pidThrKey, PSTR("THR_"), 0.1, 0, 0, 0, 20));
		addBlock(new ToServo(_hal->rc[chThr]));
	}
	virtual void update(const float & dt) {
		// read mode switch
		_hal->rc[chMode]->readRadio();
		_hal->debug->printf_P(PSTR("normalized mode: %f"), _hal->rc[chMode]->getNormalized());

		// manual
		if (_hal->rc[chMode]->getNormalized() > 0) {
			_hal->rc[chStr]->readRadio();
			_hal->rc[chThr]->readRadio();
			_hal->debug->println("manual");

		} else { // auto
			AP_Controller::update(dt);
			_hal->debug->println("automode");
		}

		//		Serial.printf("steering pwm :\t");
		//		Serial.printf("%7d\t",steeringCh->getPwm());
		//		Serial.println();

		//		Serial.printf("throttle pwm :\t");
		//		Serial.printf("%7d\t",throttleCh->getPwm());
		//		Serial.println();
	}

};

/*
class QuadController: public AP_Controller {
private:

	// position variables
	float commandNorth;
	float commandEast;
	float commandDown;
#define PID_POS_P 1
#define PID_POS_I 0
#define PID_POS_D 0
#define PID_POS_AWU 0
	float PID_POS_LIM;
	float PID_POS_LIM_MIN;
#define PID_POS_Z_P 1
#define PID_POS_Z_I 0
#define PID_POS_Z_D 0
#define PID_POS_Z_AWU 0
	float PID_POS_Z_LIM;
	float PID_POS_Z_LIM_MIN;

	// bridge variables
	float commandRoll;
	float commandPitch;
	float commandYaw;

	// attitude variables
#define PID_ATT_P 1
#define PID_ATT_I 0
#define PID_ATT_D 0
#define PID_ATT_AWU 0
#define PID_YAWPOS_P 1
#define PID_YAWPOS_I 0
#define PID_YAWPOS_D 0
#define PID_YAWPOS_AWU 0
#define PID_YAWSPEED_P 1
#define PID_YAWSPEED_I 0
#define PID_YAWSPEED_D 0
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
	float motorMin;
	float motorMax;
	onst float * heading,
	const float * velocity, const float * headingCommand,
	const float * velocityCommand, AP_RcChannel * modeCh,
	AP_RcChannel * steeringCh, AP_RcChannel *
	// motor channels
	AP_RcChannel * chMode;
	AP_RcChannel * chLeft;
	AP_RcChannel * chRight;
	AP_RcChannel * chFront;
	AP_RcChannel * chBack;
	AP_RcChannel * chRoll;
	AP_RcChannel * chPitch;
	AP_RcChannel * chYaw;
	AP_RcChannel * chThr;

	// command variables
	float * pNCmd;
	float * pN;
	float * vN;
	float * pECmd;
	float * pE;
	float * vE;
	float * pDCmd;
	float * pD;
	float * vD;

	// feedback variables
	const float * roll;
	const float * pitch;
	const float * yaw;
	float * rollRate;
	float * pitchRate;
	float * yawRate;

public:
	QuadController(AP_Var::Key cntrlKey,
			AP_Var::Key chLeftKey, AP_Var::Key chRightKey, AP_Var::Key chFrontKey, AP_Var::Key chBackKey,
			AP_Var::Key chRollKey, AP_Var::Key chPitchKey, AP_Var::Key chYawKey, AP_Var::Key chThrKey,
			AP_Var::Key pidPNKey, AP_Var::Key pidPEKey, AP_Var::Key pidPDKey,
			AP_Var::Key pidRollKey, AP_Var::Key pidPitchKey, AP_Var::Key pidYawRateKey, AP_Var::Key pidYawKey,
			const float * velocity, const float * velocityCommand,
			const float * heading, const float * headingCommand,
			const float * roll, const float * pitch, const float * yaw,
			float * rollrate, float * pitchRate, float * yawRate,
			float * pNCmd, float * pN, float * vN,
			float * pECmd, float * pE, float * vE,
			float * pDCmd, float * pD, float * vD,
			AP_RcChannel * chMode, AP_RcChannel * chLeft,
			AP_RcChannel * chRight, AP_RcChannel * chFront,
			AP_RcChannel * chBack, AP_RcChannel * chRoll,
			AP_RcChannel * chPitch, AP_RcChannel * chYaw,
			AP_RcChannel * chThr) :
	chLeft(chLeft), chRight(chRight), chFront(chFront),
	chBack(chBack), chRoll(chRoll), chPitch(chPitch),
	chYaw(chYaw), chThr(chThr)
	{
		Serial.println_P(PSTR("initializing quad controller"));

		////
		// POSITION CONTROL
		//
		// inputs:  reference north, reference east, reference down, reference yaw (user declared)
		//	    output north, output east, output down			   (output declared)
		//	    d/dt output north, d/dt output east, d/dt output down	   (feedback declared)
		// outputs: commandNorth, commandEast, commandDown, commandYaw
		//
		////

		addBlock(new SumGain(pNCmd,&one,pN,&negativeOne));
		addBlock(new PidDFB(pidPNKey, PSTR("North"), vN, PID_POS_P,PID_POS_I,PID_POS_D,PID_POS_AWU));
		addBlock(new Saturate(PID_POS_LIM_MIN,PID_POS_LIM));
		addBlock(new Sink(commandNorth));

		addBlock(new SumGain(pECmd,&one,pE,&negativeOne));
		addBlock(new PidDFB(pidPEKey, PSTR("East"), vE, PID_POS_P,PID_POS_I,PID_POS_D,PID_POS_AWU));
		addBlock(new Saturate(PID_POS_LIM_MIN,PID_POS_LIM));
		addBlock(new Sink(commandEast));
		//
		//				addBlock(new SumGain(pDCmd,&one,pD,&negativeOne));
		//				addBlock(new PidDFB(pidPDKey, PSTR("Down"),  vD, PID_POS_Z_P,PID_POS_Z_I,PID_POS_Z_D,PID_POS_Z_AWU));
		//				addBlock(new Saturate(PID_POS_Z_LIM_MIN,PID_POS_Z_LIM));
		//				addBlock(new Sink(commandDown));

		////
		// Bridge between POSITION and ATTITUDE control
		//
		// inputs:  (-commandNorth), commandEast, (-thrustMix), commandYaw
		// outputs: commandRoll, commandPitch, commandYaw, thrustMix
		//
		////

		addBlock(new Bridge(&commandNorth,&commandEast,&commandDown,&commandYaw));

		////
		// ATTITUDE CONTROL
		//
		// inputs:  commandRoll, commandPitch, commandYaw
		// outputs: rollMix, pitchMix, yawMix
		// note:    thrustMix (untouched)
		//
		////

		addBlock(new SumGain(&commandRoll,&one,roll,&negativeOne));
		addBlock(new PidDFB(pidRollKey, PSTR("ROLL"), rollRate, PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU));
		addBlock(new Sink(rollMix));

		addBlock(new SumGain(&commandPitch,&one,pitch,&negativeOne));
		addBlock(new PidDFB(pidPitchKey, PSTR("PITCH"), pitchRate, PID_ATT_P,PID_ATT_I,PID_ATT_D,PID_ATT_AWU));
		addBlock(new Sink(pitchMix));

		addBlock(new SumGain(&commandYaw,&one,yaw,&negativeOne));
		addBlock(new PidDFB(pidYawRateKey, PSTR("YAWrate"), yawRate, PID_YAWPOS_P,PID_YAWPOS_I,PID_YAWPOS_D,PID_YAWPOS_AWU));
		addBlock(new Pid(pidYawKey, PSTR("YAW"), PID_YAWSPEED_P,PID_YAWSPEED_I,PID_YAWSPEED_D,PID_YAWSPEED_AWU));
		addBlock(new Sink(yawMix));

		////
		// THRUST TRIM
		//
		// inputs:  thrustMix
		// outputs: thrustMix
		// notes: rollMix, pitchMix, yawMix (untouched)
		//
		////

		addBlock(new SumGain(THRUST_HOVER_OFFSET,&one,&thrustMix,&one));
		addBlock(new Sink(thrustMix));

		////
		// MOTOR MIX
		//
		// inputs:  rollMix, pitchMix, yawMix, thrustMix
		// outputs: (to servo: chLeft, chRight, chFront, chBack)
		//
		////

		// Left Motor
		addBlock(new SumGain(&thrustMix, &one, &rollMix, &one, &yawMix, &one));
		addBlock(new Saturate(motorMin,motorMax));
		addBlock(new ToServo(chLeft));

		// Right Motor
		addBlock(new SumGain(&thrustMix,&one,&rollMix,&negativeOne,&yawMix,&one));
		addBlock(new Saturate(motorMin,motorMax));
		addBlock(new ToServo(chRight));

		// Front Motor
		addBlock(new SumGain(&thrustMix,&one,&pitchMix,&one,&yawMix,&negativeOne));
		addBlock(new Saturate(motorMin,motorMax));
		addBlock(new ToServo(chFront));

		// Back Motor
		addBlock(new SumGain(&thrustMix,&one,&pitchMix,&negativeOne,&yawMix,&negativeOne));
		addBlock(new Saturate(motorMin,motorMax));
		addBlock(new ToServo(chBack));
	}

};
void controllerInit() {
	_rc.push_back(new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 45));
	_rc.push_back(new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 0, 1));
	_rc.push_back(new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 1, 1));
	_rc.push_back(new AP_RcChannel(k_chFront, PSTR("FRONT_"), APM_RC, 2, 1));
	_rc.push_back(new AP_RcChannel(k_chBack, PSTR("BACK_"), APM_RC, 3, 1));

	_controller = new QuadController(k_cntrl,
			k_chLeft, k_chRight, k_chFront, k_chBack,
			k_chRoll, k_chPitch, k_chYaw, k_chThr,
			k_pidPN, k_pidPE, k_pidPD,
			k_pidRoll, k_pidPitch, k_pidYawRate, k_pidYaw,
			&navigator()->groundSpeed, &guide()->groundSpeedCommand,
			&navigator()->yaw, &guide()->headingCommand,
			&navigator()->roll, &navigator()->pitch, &navigator()->yaw,
			&navigator()->rollRate, &navigator()->pitchRate, &navigator()->yawRate,
			&guide()->pNCmd, &navigator()->pN, &navigator()->vN,
			&guide()->pECmd, &navigator()->pE, &navigator()->vE,
			&guide()->pDCmd, &navigator()->pD, &navigator()->vD,
			_rc[0], _rc[1], _rc[2], _rc[3], _rc[4],
			_rc[5], _rc[6], _rc[7], _rc[8]);
}
*/

/*
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
*/

////
// AP_Controller.h extended: functions for "BRIDGE", "PIDDFB", and "QUADMIX"
////

/// Bridge Block
class Bridge: public AP_Controller::Block {
public:
	Bridge(float * commandNorth, float * commandEast, float * commandDown,
			float * commandYaw) :
		_cNorth(commandNorth), _cEast(commandEast), _cDown(commandDown),
				_cYaw(commandYaw) {
		_output.push_back(new float(0.0));
		_output.push_back(new float(0.0));
		_output.push_back(new float(0.0));
		_output.push_back(new float(0.0));
	}
	virtual void update(const float & dt) {
		// multiply (commandNorth * gain negativeOne, commandEast * gain one
		//			 commandDown  * gain negativeOne, commandYaw  * gain one)
		(*_cNorth) = (-1) * (*_cNorth);
		(*_cDown) = (-1) * (*_cDown);
		// "transform-to-body"
		float trigSin = sin((*_cYaw) * (-1));
		float trigCos = cos((*_cYaw) * (-1));
		float commandPitch = (*_cEast) * trigCos - (*_cNorth) * trigSin;
		float commandRoll = (*_cEast) * trigSin + (*_cNorth) * trigCos;

		// "thrust-trim-adjust"
		float thrustMix = (*_cDown);
		if (fabs(commandRoll) > 0.5) {
			thrustMix = thrustMix * 1.13949393;
		} else {
			thrustMix = thrustMix / cos(commandRoll);
		}
		if (fabs(commandPitch) > 0.5) {
			thrustMix = thrustMix * 1.13949393;
		} else {
			thrustMix = thrustMix / cos(commandPitch);
		}

		// "mix manual"
		commandRoll -= (*_ATT_OFFSET_X) * (*_MIX_OFFSET_WEIGHT);
		commandPitch -= (*_ATT_OFFSET_Y) * (*_MIX_OFFSET_WEIGHT);
		thrustMix -= (*_ATT_OFFSET_Z) * (*_MIX_OFFSET_WEIGHT);

		commandRoll += (_chRoll ->getPosition()) * (*_MIX_REMOTE_WEIGHT);
		commandPitch += (_chPitch->getPosition()) * (*_MIX_REMOTE_WEIGHT);
		(*_cYaw) += (_chYaw ->getPosition()) * (*_MIX_REMOTE_WEIGHT);
		thrustMix += (_chThr ->getPosition()) * (*_MIX_REMOTE_WEIGHT);

		output(0) = commandRoll;
		output(1) = commandPitch;
		output(2) = (*_cYaw);
		output(3) = thrustMix;
	}
private:
	float * _cNorth;
	float * _cEast;
	float * _cDown;
	float * _cYaw;
	AP_RcChannel * _chRoll;
	AP_RcChannel * _chPitch;
	AP_RcChannel * _chYaw;
	AP_RcChannel * _chThr;
	float * _ATT_OFFSET_X;
	float * _ATT_OFFSET_Y;
	float * _ATT_OFFSET_Z;
	float * _ATT_OFFSET_WEIGHT;
	float * _MIX_OFFSET_WEIGHT;
	float * _MIX_REMOTE_WEIGHT;
};


} // namespace apo

#endif // defaultControllers_H
// vim:ts=4:sw=4:expandtab
