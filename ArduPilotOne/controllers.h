#ifndef defaultControllers_H
#define defaultControllers_H

//#include "ArduPilotOne.h"
#include "AP_Controller.h"
#include "AP_RcChannelSimple.h"
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
			AP_Var::Key pidThrKey, AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
		AP_Controller(nav, guide, hal), _group(cntrlKey, PSTR("CNTRL_")),
				_mode(&_group, 1, 0, PSTR("MODE")) {
		_hal->debug->println_P(PSTR("initializing car controller"));

		// NOTE, if you change this order, change the numbers above
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chMode, PSTR("MODE_"), APM_RC, 7, 1));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chStr, PSTR("STR_"), APM_RC, 0, 45));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chThr, PSTR("THR_"), APM_RC, 1, 100));

		// steering control loop
		addBlock(
				new SumGain(&(_guide->headingCommand), &one, &(_nav->yaw),
						&negativeOne));
		addBlock(new Pid(pidStrKey, PSTR("STR_"), 1, 0, 0, 0, 20));
		addBlock(new ToServo(_hal->rc[chMode])); // index depends on order of channels pushed back into _hal->rc

		// throttle control loop
		addBlock(
				new SumGain(&(_guide->groundSpeedCommand), &one,
						&(_nav->groundSpeed), &negativeOne));
		addBlock(new Pid(pidThrKey, PSTR("THR_"), 0.1, 0, 0, 0, 20));
		addBlock(new ToServo(_hal->rc[chThr]));
	}
	virtual void update(const float & dt) {
		// read mode switch
		//_hal->debug->println_P(PSTR("update loop"));
		_hal->rc[chMode]->readRadio();
		//_hal->debug->printf_P(PSTR("normalized mode: %f"), _hal->rc[chMode]->getNormalized());

		// manual
		if (_hal->rc[chMode]->getPosition() > 0) {
			_hal->rc[chStr]->readRadio();
			_hal->rc[chThr]->readRadio();
			//_hal->debug->println("manual");

		} else { // auto
			AP_Controller::update(dt);
			//_hal->debug->println("automode");
		}

		//		Serial.printf("steering pwm :\t");
		//		Serial.printf("%7d\t",steeringCh->getPwm());
		//		Serial.println();

		//		Serial.printf("throttle pwm :\t");
		//		Serial.pr intf("%7d\t",throttleCh->getPwm());
		//		Serial.println();
	}

};

#if CUSTOM_INCLUDES == CUSTOM_MIKROKOPTER
#include "mikrokopter.h"
#endif

class QuadController: public AP_Controller {
public:

	/**
	 * note that these are not the controller radio channel numbers, they are just
	 * unique keys so they can be reaccessed from the hal rc vector
	 */
	enum autoChannel_t {
		CH_MODE = 0, // note scicoslab channels set mode, left, right, front, back order
		CH_LEFT, // this enum must match this
		CH_RIGHT,
		CH_FRONT,
		CH_BACK,
		CH_ROLL,
		CH_PITCH,
		CH_YAW,
		CH_THRUST
	};

	class Bridge: public Block {
	public:
		Bridge(QuadController * controller) :
			_controller(controller) {
		}
		virtual void update(const float & dt) {
			// "transform-to-body"
			{
				float trigSin = sin(-_controller->_nav->yaw);
				float trigCos = cos(-_controller->_nav->yaw);
				_controller->_cmdPitch = _controller->_cmdEastTilt * trigCos
						- _controller->_cmdNorthTilt * trigSin;
				_controller->_cmdRoll = -_controller->_cmdEastTilt * trigSin
						+ _controller->_cmdNorthTilt * trigCos;
				// note that the north tilt is negative of the pitch
			}

			// "thrust-trim-adjust"
			if (fabs(_controller->_cmdRoll) > 0.5) {
				_controller->_thrustMix *= 1.13949393;
			} else {
				_controller->_thrustMix /= cos(_controller->_cmdRoll);
			}
			if (fabs(_controller->_cmdPitch) > 0.5) {
				_controller->_thrustMix *= 1.13949393;
			} else {
				_controller->_thrustMix /= cos(_controller->_cmdPitch);
			}

			// "mix manual"
			/*
			_controller->_cmdRoll -= _controller->_attOffsetX
					* _controller->_mixOffsetWeight;
			_controller->_cmdPitch -= _controller->_attOffsetY
					* _controller->_mixOffsetWeight;
			_controller->_thrustMix -= _controller->_attOffsetZ
					* _controller->_mixOffsetWeight;

			_controller->_cmdRoll
					+= _controller->_hal->rc[CH_ROLL]->getPosition()
							* _controller->_mixRemoteWeight;
			_controller->_cmdPitch
					+= _controller->_hal->rc[CH_PITCH]->getPosition()
							* _controller->_mixRemoteWeight;
			_controller->_cmdYaw
					+= _controller->_hal->rc[CH_YAW]->getPosition()
							* _controller->_mixRemoteWeight;
			_controller->_thrustMix
					+= _controller->_hal->rc[CH_THRUST]->getPosition()
							* _controller->_mixRemoteWeight;
							*/
		}
	private:
		QuadController * _controller;
	};

	QuadController(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
		AP_Controller(nav, guide, hal), _thrustMixTrim(THRUST_HOVER_OFFSET),
				_cmdNorthTilt(0), _cmdEastTilt(0), _cmdRoll(0), _cmdPitch(0),
				_cmdYaw(0), _thrustMix(0), _rollMix(0), _yawMix(0),
				_pitchMix(0), _mixOffsetWeight(MIX_OFFSET_WEIGHT),
				_mixRemoteWeight(MIX_REMOTE_WEIGHT), _attOffsetX(ATT_OFFSET_X),
				_attOffsetY(ATT_OFFSET_Y), _attOffsetZ(ATT_OFFSET_Z) {

		_hal->rc[CH_MODE]->readRadio();
		//_hal->debug->printf_P(PSTR("normalized mode: %f"), _hal->rc[chMode]->getNormalized());

		// manual
		if (_hal->rc[CH_MODE]->getPosition() > 0) {
			_mixRemoteWeight = 1;
		} else { // auto
			_mixRemoteWeight = 0;
		}

		/*
		 * allocate radio channels
		 */
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chMode, PSTR("MODE_"), APM_RC, 7));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chRoll, PSTR("ROLL_"), APM_RC, 0));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chPitch, PSTR("PITCH_"), APM_RC, 1));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chYaw, PSTR("YAW_"), APM_RC, 2));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chThr, PSTR("THRUST_"), APM_RC, 3));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chLeft, PSTR("LEFT_"), APM_RC, 0));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chRight, PSTR("RIGHT_"), APM_RC, 1));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chFront, PSTR("FRONT_"), APM_RC, 2));
		_hal->rc.push_back(
				new AP_RcChannelSimple(k_chBack, PSTR("BACK_"), APM_RC, 3));

		/*
		 * position loop
		 */

		// north position error -> north tilt
		addBlock(new SumGain(&(_guide->pNCmd), &one, &(_nav->pN), &negativeOne));
		addBlock(
				new PidDFB(k_pidPN, PSTR("NORTH_"), &(_nav->vN), PID_ATT_P,
						PID_ATT_I, PID_ATT_D, PID_ATT_AWU, PID_POS_LIM));
		addBlock(new Sink(_cmdNorthTilt));

		// east position error -> east tilt
		addBlock(new SumGain(&(_guide->pECmd), &one, &(_nav->pE), &negativeOne));
		addBlock(
				new PidDFB(k_pidPE, PSTR("EAST_"), &(_nav->vE), PID_ATT_P,
						PID_ATT_I, PID_ATT_D, PID_ATT_AWU, PID_POS_LIM));
		addBlock(new Sink(_cmdEastTilt));

		// down error -> -thrust mix
		addBlock(new SumGain(&(_guide->pDCmd), &one, &(_nav->pD), &negativeOne));
		addBlock(
				new PidDFB(k_pidPD, PSTR("DOWN_"), &(_nav->vD), PID_POS_Z_P,
						PID_POS_Z_I, PID_POS_Z_D, PID_POS_Z_AWU, PID_POS_Z_LIM));
		addBlock(new Sink(_thrustMix));

		/*
		 * bridge, rotation of north/east tilt to body frame
		 * manual control mixing
		 * trim thrust mix for hover
		 */
		addBlock(new Bridge(this));

		/*
		 * attitude loop
		 */

		// roll error -> roll mix
		addBlock(new SumGain(&_cmdRoll, &one, &(_nav->roll), &negativeOne));
		addBlock(
				new PidDFB(k_pidRoll, PSTR("ROLL_"), &(_nav->rollRate),
						PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU,
						PID_ATT_LIM));
		addBlock(new Sink(_rollMix));

		// pitch error -> pitch mix
		addBlock(new SumGain(&_cmdPitch, &one, &(_nav->pitch), &negativeOne));
		addBlock(
				new PidDFB(k_pidPitch, PSTR("PITCH_"), &(_nav->pitchRate),
						PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU,
						PID_ATT_LIM));
		addBlock(new Sink(_pitchMix));

		// yaw error -> yaw mix
		addBlock(new SumGain(&_cmdYaw, &one, &(_nav->yaw), &negativeOne));
		addBlock(
				new PidDFB(k_pidYawRate, PSTR("YAWRATE_"), &(_nav->yawRate),
						PID_YAWSPEED_P, PID_YAWSPEED_I, PID_YAWSPEED_D,
						PID_YAWSPEED_AWU, PID_YAWSPEED_LIM));
		addBlock(
				new Pid(k_pidYaw, PSTR("YAW_"), PID_YAWPOS_P, PID_YAWPOS_I,
						PID_YAWPOS_D, PID_YAWPOS_AWU, PID_YAWPOS_LIM));
		addBlock(new Sink(_yawMix));

		/*
		 * thrust trim
		 */

		// note that the position D -> thrust -1 gain is applied here to the
		// thrust mix
		addBlock(new SumGain(&_thrustMixTrim, &one, &_thrustMix, &negativeOne));
		//addBlock(new SumGain(&_thrustMix, &negativeOne)); // XXX, this is wrong but sorta works, why?
		addBlock(new Sink(_thrustMix));

		/*
		 * motor mix
		 */

		// left
		addBlock(
				new SumGain(&_thrustMix, &one, &_rollMix, &one, &_yawMix, &one));
		addBlock(new ToServo(_hal->rc[CH_LEFT]));

		// right
		addBlock(
				new SumGain(&_thrustMix, &one, &_rollMix, &negativeOne,
						&_yawMix, &one));
		addBlock(new ToServo(_hal->rc[CH_RIGHT]));

		// front
		addBlock(
				new SumGain(&_thrustMix, &one, &_pitchMix, &one, &_yawMix,
						&negativeOne));
		addBlock(new ToServo(_hal->rc[CH_FRONT]));

		// back
		addBlock(
				new SumGain(&_thrustMix, &one, &_pitchMix, &negativeOne,
						&_yawMix, &negativeOne));
		addBlock(new ToServo(_hal->rc[CH_BACK]));
	}
	virtual void update(const float & dt) {
		AP_Controller::update(dt);
		/*
		_hal->debug->printf_P(
				PSTR("Position Loop: North, East, Down: %f %f %f\n"),
				_cmdNorthTilt, _cmdEastTilt, _thrustMix);
		_hal->debug->printf_P(
				PSTR("Attitude Loop: RollMix, PitchMix, YawMix: %f %f %f\n"),
				_rollMix, _pitchMix, _yawMix);
		_hal->debug->printf_P(PSTR("thrustMixTrim: %f thrustMix: %f\n"), _thrustMixTrim, _thrustMix);
		_hal->debug->printf_P(
				PSTR("CH_LEFT, CH_RIGHT, CH_FRONT, CH_BACK: %f %f %f %f\n"),
				_hal->rc[CH_LEFT]->getPosition(),
				_hal->rc[CH_RIGHT]->getPosition(),
				_hal->rc[CH_FRONT]->getPosition(),
				_hal->rc[CH_BACK]->getPosition());
		*/
	}
private:

	float _thrustMixTrim;

	float _cmdNorthTilt;
	float _cmdEastTilt;

	float _cmdRoll;
	float _cmdPitch;
	float _cmdYaw;

	float _thrustMix;
	float _rollMix;
	float _yawMix;
	float _pitchMix;

	float _mixOffsetWeight;
	float _mixRemoteWeight;

	float _attOffsetX;
	float _attOffsetY;
	float _attOffsetZ;
};

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
 addCh(new AP_RcChannelSimple(chRollKey,PSTR("ROLL"),APM_RC,chRoll,45));
 addCh(new AP_RcChannelSimple(chPitchKey,PSTR("PTCH"),APM_RC,chPitch,45));
 addCh(new AP_RcChannelSimple(chYawKey,PSTR("YAW"),APM_RC,chYaw,45));

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

} // namespace apo

#endif // defaultControllers_H
// vim:ts=4:sw=4:expandtab
