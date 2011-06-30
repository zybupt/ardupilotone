/*
 * controllers.h
 *
 *  Created on: Jun 28, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

#include "../APO/AP_Controller.h"

namespace apo {

class PlaneController: public AP_Controller {
private:
	AP_Var_group _group;
	AP_Var_group _trimGroup;AP_Uint8 _mode;AP_Uint8 _rdrAilMix;
	bool _needsTrim;
	AP_Float _ailTrim;
	AP_Float _elvTrim;
	AP_Float _rdrTrim;
	AP_Float _thrTrim;
	enum {
		ch_mode = 0, ch_roll, ch_pitch, ch_yaw, ch_thr
	};
	enum {
		k_chMode = k_radioChannelsStart,
		k_chRoll,
		k_chPitch,
		k_chYaw,
		k_chThr,

		k_pidBnkRll = k_controllersStart,
		k_pidSpdPit,
		k_pidPitPit,
		k_pidYwrYaw,
		k_pidHdgBnk,
		k_pidAltThr,

		k_trim = k_customStart
	};
	BlockPID pidBnkRll; // bank error to roll servo deflection
	BlockPID pidSpdPit; // speed error to pitch command
	BlockPID pidPitPit; // pitch error to pitch servo deflection
	BlockPID pidYwrYaw; // yaw rate error to yaw servo deflection
	BlockPID pidHdgBnk; // heading error to bank command
	BlockPID pidAltThr; // altitude error to throttle deflection

public:
	PlaneController(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				_group(k_cntrl, PSTR("cntrl_")),
				_trimGroup(k_trim, PSTR("trim_")),
				_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("mode")),
				_rdrAilMix(&_group, 2, rdrAilMix, PSTR("rdrAilMix")),
				_needsTrim(false),
				_ailTrim(&_trimGroup, 1, ailTrim, PSTR("ail")),
				_elvTrim(&_trimGroup, 2, elvTrim, PSTR("elv")),
				_rdrTrim(&_trimGroup, 3, rdrTrim, PSTR("rdr")),
				_thrTrim(&_trimGroup, 4, thrTrim, PSTR("thr")),
				pidBnkRll(new AP_Var_group(k_pidBnkRll, PSTR("bnkRll_")), 1,
						pidBnkRllP, pidBnkRllI, pidBnkRllD, pidBnkRllAwu,
						pidBnkRllLim, pidBnkRllDFCut),
				pidPitPit(new AP_Var_group(k_pidPitPit, PSTR("pitPit_")), 1,
						pidPitPitP, pidPitPitI, pidPitPitD, pidPitPitAwu,
						pidPitPitLim, pidPitPitDFCut),
				pidSpdPit(new AP_Var_group(k_pidSpdPit, PSTR("spdPit_")), 1,
						pidSpdPitP, pidSpdPitI, pidSpdPitD, pidSpdPitAwu,
						pidSpdPitLim, pidSpdPitDFCut),
				pidYwrYaw(new AP_Var_group(k_pidYwrYaw, PSTR("ywrYaw_")), 1,
						pidYwrYawP, pidYwrYawI, pidYwrYawD, pidYwrYawAwu,
						pidYwrYawLim, pidYwrYawDFCut),
				pidHdgBnk(new AP_Var_group(k_pidHdgBnk, PSTR("hdgBnk_")), 1,
						pidHdgBnkP, pidHdgBnkI, pidHdgBnkD, pidHdgBnkAwu,
						pidHdgBnkLim, pidHdgBnkDFCut),
				pidAltThr(new AP_Var_group(k_pidAltThr, PSTR("altThr_")), 1,
						pidAltThrP, pidAltThrI, pidAltThrD, pidAltThrAwu,
						pidAltThrLim, pidAltThrDFCut) {

		_hal->debug->println_P(PSTR("initializing car controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("mode_"), APM_RC, 5, 1100,
						1500, 1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRoll, PSTR("roll_"), APM_RC, 0, 1200,
						1500, 1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chPitch, PSTR("pitch_"), APM_RC, 1, 1200,
						1500, 1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chYaw, PSTR("yaw_"), APM_RC, 2, 1200, 1500,
						1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("thr_"), APM_RC, 3, 1100, 1100,
						1900, RC_MODE_INOUT));
	}
	virtual MAV_MODE getMode() {
		return (MAV_MODE) _mode.get();
	}
	virtual void update(const float & dt) {

		if (_hal->heartBeatLost()) {
			// check for heartbeat
			_mode = MAV_MODE_FAILSAFE;
			setAllRadioChannelsToNeutral();
			_hal->setState(MAV_STATE_EMERGENCY);
			_hal->debug->printf_P(PSTR("comm lost, send heartbeat from gcs\n"));
			return;
		} else if (_hal->rc[ch_thr]->getRadioPosition() < 0.05) {
			// if the value of the throttle is less than 5% cut motor power
			_mode = MAV_MODE_LOCKED;
			setAllRadioChannelsToNeutral();
			_hal->setState(MAV_STATE_STANDBY);
			return;
		} else if (_hal->getMode() == MODE_LIVE) {
			_hal->setState(MAV_STATE_ACTIVE);
		} else if (_hal->getMode() == MODE_HIL_CNTL) {
			_hal->setState(MAV_STATE_HILSIM);
		}

		// read switch to set mode
		if (_hal->rc[ch_mode]->getRadioPosition() > 0) {
			_mode = MAV_MODE_MANUAL;
		} else {
			_mode = MAV_MODE_AUTO;
		}

		// manual mode
		switch (_mode) {

		case MAV_MODE_MANUAL: {
			setAllRadioChannelsManually();

			// force auto to read new manual trim
			if (_needsTrim == false)
				_needsTrim = true;
			//_hal->debug->println("manual");
			break;
		}
		case MAV_MODE_AUTO: {
			float headingError = _guide->getHeadingCommand()
					- _nav->getHeading();
			if (headingError > 180 * deg2Rad)
				headingError -= 360 * deg2Rad;
			if (headingError < -180 * deg2Rad)
				headingError += 360 * deg2Rad;

			float aileron = pidBnkRll.update(
					pidHdgBnk.update(headingError, dt) - _nav->getRoll(), dt);
			float elevator = pidPitPit.update(
					-pidSpdPit.update(
							_guide->getAirSpeedCommand() - _nav->getAirSpeed(),
							dt) - _nav->getPitch(), dt);
			float rudder = pidYwrYaw.update(-_nav->getYawRate(), dt);
			// desired yaw rate is zero, needs washout
			float throttle = pidAltThr.update(
					_guide->getAltitudeCommand() - _nav->getAlt(), dt);

			// if needs trim
			if (_needsTrim) {
				// need to subtract current controller deflections so control
				// surfaces are actually at the same position as manual flight
				_ailTrim = _hal->rc[ch_roll]->getRadioPosition() - aileron;
				_elvTrim = _hal->rc[ch_pitch]->getRadioPosition() - elevator;
				_rdrTrim = _hal->rc[ch_yaw]->getRadioPosition() - rudder;
				_thrTrim = _hal->rc[ch_thr]->getRadioPosition() - throttle;
				_needsTrim = false;
			}

			// actuator mixing/ output
			_hal->rc[ch_roll]->setPosition(
					aileron + _rdrAilMix * rudder + _ailTrim);
			_hal->rc[ch_yaw]->setPosition(rudder + _rdrTrim);
			_hal->rc[ch_pitch]->setPosition(elevator + _elvTrim);
			_hal->rc[ch_thr]->setPosition(throttle + _thrTrim);

			//_hal->debug->println("automode");
			break;
		}

		default: {
			setAllRadioChannelsToNeutral();
			_mode = MAV_MODE_FAILSAFE;
			_hal->setState(MAV_STATE_EMERGENCY);
			_hal->debug->printf_P(PSTR("unknown controller mode\n"));
			break;
		}

		}
	}

	class BoatController: public AP_Controller {
	private:
		AP_Var_group _group;AP_Uint8 _mode;
		enum {
			k_chMode = k_radioChannelsStart, k_chStr, k_chThr
		};
		enum {
			k_pidStr = k_controllersStart, k_pidThr
		};
		enum {
			CH_MODE = 0, CH_STR, CH_THR
		};
		BlockPIDDfb pidStr;
		BlockPID pidThr;
	public:
		BoatController(AP_Navigator * nav, AP_Guide * guide,
				AP_HardwareAbstractionLayer * hal) :
					AP_Controller(nav, guide, hal),
					_group(k_cntrl, PSTR("CNTRL_")),
					_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("MODE")),
					pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1,
							steeringP, steeringI, steeringD, steeringIMax,
							steeringYMax),
					pidThr(new AP_Var_group(k_pidThr, PSTR("THR_")), 1,
							throttleP, throttleI, throttleD, throttleIMax,
							throttleYMax, throttleDFCut) {
			_hal->debug->println_P(PSTR("initializing boat controller"));

			_hal->rc.push_back(
					new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 1100,
							1500, 1900));
			_hal->rc.push_back(
					new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0, 1100,
							1540, 1900));
			_hal->rc.push_back(
					new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1, 1100,
							1500, 1900));
		}
		virtual MAV_MODE getMode() {
			return (MAV_MODE) _mode.get();
		}
		virtual void update(const float & dt) {

			// check for heartbeat
			if (_hal->heartBeatLost()) {
				_mode = MAV_MODE_FAILSAFE;
				setAllRadioChannelsToNeutral();
				_hal->setState(MAV_STATE_EMERGENCY);
				_hal->debug->printf_P(PSTR("comm lost, send heartbeat from gcs\n"));
				return;
			} else if (_hal->rc[CH_THR]->getPosition() < 0.05) {
				// if throttle less than 5% cut motor power
				_mode = MAV_MODE_LOCKED;
				setAllRadioChannelsToNeutral();
				_hal->setState(MAV_STATE_STANDBY);
				return;
			} else if (_hal->getMode() == MODE_LIVE) {
				_hal->setState(MAV_STATE_ACTIVE);
			} else if (_hal->getMode() == MODE_HIL_CNTL) {
				_hal->setState(MAV_STATE_HILSIM);
			}

			// read switch to set mode
			_hal->rc[CH_MODE]->setPwm(_hal->rc[CH_MODE]->readRadio());
			if (_hal->rc[CH_MODE]->getPosition() > 0) {
				_mode = MAV_MODE_MANUAL;
			} else {
				_mode = MAV_MODE_AUTO;
			}

			// manual mode
			switch (_mode) {

			case MAV_MODE_MANUAL: {
				setAllRadioChannelsManually();
				//_hal->debug->println("manual");
				break;
			}
			case MAV_MODE_AUTO: {
				float headingError = _guide->getHeadingCommand()
						- _nav->getHeading();
				if (headingError > 180 * deg2Rad)
					headingError -= 360 * deg2Rad;
				if (headingError < -180 * deg2Rad)
					headingError += 360 * deg2Rad;
				_hal->rc[CH_STR]->setPosition(
						pidStr.update(headingError, _nav->getYawRate(), dt));
				_hal->rc[CH_THR]->setPosition(
						pidThr.update(
								_guide->getGroundSpeedCommand()
										- _nav->getGroundSpeed(), dt));
				//_hal->debug->println("automode");
				break;
			}

			default: {
				setAllRadioChannelsToNeutral();
				_mode = MAV_MODE_FAILSAFE;
				_hal->setState(MAV_STATE_EMERGENCY);
				_hal->debug->printf_P(PSTR("unknown controller mode\n"));
				break;
			}

			}
		}
	};

	class QuadController: public AP_Controller {
	public:

		/**
		 * note that these are not the controller radio channel numbers, they are just
		 * unique keys so they can be reaccessed from the hal rc vector
		 */
		enum {
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

		enum {
			k_chMode = k_radioChannelsStart,
			k_chLeft,
			k_chRight,
			k_chFront,
			k_chBack,
			k_chRoll,
			k_chPitch,
			k_chYaw,
			k_chThr
		};

		enum {
			k_pidGroundSpeed2Throttle = k_controllersStart,
			k_pidStr,
			k_pidPN,
			k_pidPE,
			k_pidPD,
			k_pidRoll,
			k_pidPitch,
			k_pidYawRate,
			k_pidYaw,
		};

		QuadController(AP_Navigator * nav, AP_Guide * guide,
				AP_HardwareAbstractionLayer * hal) :
					AP_Controller(nav, guide, hal),
					pidRoll(new AP_Var_group(k_pidRoll, PSTR("ROLL_")), 1,
							PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU,
							PID_ATT_LIM),
					pidPitch(new AP_Var_group(k_pidPitch, PSTR("PITCH_")), 1,
							PID_ATT_P, PID_ATT_I, PID_ATT_D, PID_ATT_AWU,
							PID_ATT_LIM),
					pidYaw(new AP_Var_group(k_pidYaw, PSTR("YAW_")), 1,
							PID_YAWPOS_P, PID_YAWPOS_I, PID_YAWPOS_D,
							PID_YAWPOS_AWU, PID_YAWPOS_LIM),
					pidYawRate(new AP_Var_group(k_pidYawRate, PSTR("YAWRT_")),
							1, PID_YAWSPEED_P, PID_YAWSPEED_I, PID_YAWSPEED_D,
							PID_YAWSPEED_AWU, PID_YAWSPEED_LIM,
							PID_YAWSPEED_DFCUT),
					pidPN(new AP_Var_group(k_pidPN, PSTR("NORTH_")), 1,
							PID_POS_P, PID_POS_I, PID_POS_D, PID_POS_AWU,
							PID_POS_LIM),
					pidPE(new AP_Var_group(k_pidPE, PSTR("EAST_")), 1,
							PID_POS_P, PID_POS_I, PID_POS_D, PID_POS_AWU,
							PID_POS_LIM),
					pidPD(new AP_Var_group(k_pidPD, PSTR("DOWN_")), 1,
							PID_POS_Z_P, PID_POS_Z_I, PID_POS_Z_D,
							PID_POS_Z_AWU, PID_POS_Z_LIM) {
			/*
			 * allocate radio channels
			 * the order of the channels has to match the enumeration above
			 */
			_hal->rc.push_back(
					new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 5, 1100,
							1500, 1900, RC_MODE_IN));
			_hal->rc.push_back(
					new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 0, 1100,
							1100, 1900, RC_MODE_OUT));
			_hal->rc.push_back(
					new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 1,
							1100, 1100, 1900, RC_MODE_OUT));
			_hal->rc.push_back(
					new AP_RcChannel(k_chFront, PSTR("FRONT_"), APM_RC, 2,
							1100, 1100, 1900, RC_MODE_OUT));
			_hal->rc.push_back(
					new AP_RcChannel(k_chBack, PSTR("BACK_"), APM_RC, 3, 1100,
							1100, 1900, RC_MODE_OUT));
			_hal->rc.push_back(
					new AP_RcChannel(k_chRoll, PSTR("ROLL_"), APM_RC, 0, 1100,
							1500, 1900, RC_MODE_IN));
			_hal->rc.push_back(
					new AP_RcChannel(k_chPitch, PSTR("PITCH_"), APM_RC, 1,
							1100, 1500, 1900, RC_MODE_IN));
			_hal->rc.push_back(
					new AP_RcChannel(k_chYaw, PSTR("YAW_"), APM_RC, 2, 1100,
							1500, 1900, RC_MODE_IN));
			_hal->rc.push_back(
					new AP_RcChannel(k_chThr, PSTR("THRUST_"), APM_RC, 3, 1100,
							1100, 1900, RC_MODE_IN));
		}

		virtual void update(const float & dt) {

			// read and set pwm so they can be read as positions later
			_hal->rc[CH_MODE]->setUsingRadio();
			_hal->rc[CH_ROLL]->setUsingRadio();
			_hal->rc[CH_PITCH]->setUsingRadio();
			_hal->rc[CH_YAW]->setUsingRadio();
			_hal->rc[CH_THRUST]->setUsingRadio();

			if (_hal->heartBeatLost()) {
				// heartbeat lost, go to failsafe mode
				_mode = MAV_MODE_FAILSAFE;
				setAllRadioChannelsToNeutral();
				_hal->setState(MAV_STATE_EMERGENCY);
				_hal->debug->printf_P(PSTR("comm lost, send heartbeat from gcs\n"));
				return;
			} else if (_hal->rc[CH_THRUST]->getPosition() < 0.05) {
				// if throttle less than 5% cut motor power
				_mode = MAV_MODE_LOCKED;
				setAllRadioChannelsToNeutral();
				_hal->setState(MAV_STATE_STANDBY);
				return;
			} else if (_hal->getMode() == MODE_LIVE) {
				_hal->setState(MAV_STATE_ACTIVE);
			} else if (_hal->getMode() == MODE_HIL_CNTL) {
				_hal->setState(MAV_STATE_HILSIM);
			}

			// manual mode
			float mixRemoteWeight = 0;
			if (_hal->rc[CH_MODE]->getPwm() > 1350) {
				mixRemoteWeight = 1;
				_mode = MAV_MODE_MANUAL;
			} else {
				_mode = MAV_MODE_AUTO;
			}

			// "mix manual"
			float cmdRoll = 0.5 * _hal->rc[CH_ROLL]->getPosition()
					* mixRemoteWeight;
			float cmdPitch = 0.5 * _hal->rc[CH_PITCH]->getPosition()
					* mixRemoteWeight;
			float cmdYawRate = 0.5 * _hal->rc[CH_YAW]->getPosition()
					* mixRemoteWeight;
			float thrustMix = _hal->rc[CH_THRUST]->getPosition()
					* mixRemoteWeight;

			// position loop
			/*
			 float cmdNorthTilt = pidPN.update(_nav->getPN(),_nav->getVN(),dt);
			 float cmdEastTilt = pidPE.update(_nav->getPE(),_nav->getVE(),dt);
			 float cmdDown = pidPD.update(_nav->getPD(),_nav->getVD(),dt);

			 // "transform-to-body"
			 {
			 float trigSin = sin(-yaw);
			 float trigCos = cos(-yaw);
			 _cmdPitch = _cmdEastTilt * trigCos
			 - _cmdNorthTilt * trigSin;
			 _cmdRoll = -_cmdEastTilt * trigSin
			 + _cmdNorthTilt * trigCos;
			 // note that the north tilt is negative of the pitch
			 }

			 //thrustMix += THRUST_HOVER_OFFSET;

			 // "thrust-trim-adjust"
			 if (fabs(_cmdRoll) > 0.5) {
			 _thrustMix *= 1.13949393;
			 } else {
			 _thrustMix /= cos(_cmdRoll);
			 }
			 if (fabs(_cmdPitch) > 0.5) {
			 _thrustMix *= 1.13949393;
			 } else {
			 _thrustMix /= cos(_cmdPitch);
			 }
			 */

			// attitude loop
			// XXX negative sign added to nav roll, not sure why this is necessary
			// XXX negative sign added to nav roll rate, not sure why this is necessary
			float rollMix = pidRoll.update(cmdRoll + _nav->getRoll(),
					-_nav->getRollRate(), dt);
			// XXX negative sign added to cmdPitch, not sure why this is necessary
			float pitchMix = pidPitch.update(-cmdPitch - _nav->getPitch(),
					_nav->getPitchRate(), dt);
			// XXX negative sign added to cmdYawRate, not sure why this is necessary
			float yawMix = pidYawRate.update(-cmdYawRate - _nav->getYawRate(),
					dt);

			_hal->rc[CH_LEFT]->setPosition(thrustMix + rollMix + yawMix);
			_hal->rc[CH_RIGHT]->setPosition(thrustMix - rollMix + yawMix);
			_hal->rc[CH_FRONT]->setPosition(thrustMix + pitchMix - yawMix);
			_hal->rc[CH_BACK]->setPosition(thrustMix - pitchMix - yawMix);

			//		_hal->debug->printf("L: %f\t R: %f\t F: %f\t B: %f\n",
			//				_hal->rc[CH_LEFT]->getPosition(),
			//				_hal->rc[CH_RIGHT]->getPosition(),
			//				_hal->rc[CH_FRONT]->getPosition(),
			//				_hal->rc[CH_BACK]->getPosition());

			_hal->debug->printf(
					"rollMix: %f\t pitchMix: %f\t yawMix: %f\t thrustMix: %f\n",
					rollMix, pitchMix, yawMix, thrustMix);

			//			_hal->debug->printf("roll pwm: %d\t pitch pwm: %d\t yaw pwm: %d\t thrust pwm: %d\n",
			//					_hal->rc[CH_ROLL]->readRadio(),
			//					_hal->rc[CH_PITCH]->readRadio(),
			//					_hal->rc[CH_YAW]->readRadio(),
			//					_hal->rc[CH_THRUST]->readRadio());
		}
		virtual MAV_MODE getMode() {
			return (MAV_MODE) _mode.get();
		}
	private:
		AP_Uint8 _mode;
		BlockPIDDfb pidRoll, pidPitch, pidYaw;
		BlockPID pidYawRate;
		BlockPIDDfb pidPN, pidPE, pidPD;

	};
};

class CarController: public AP_Controller {
private:
	AP_Var_group _group;AP_Uint8 _mode;
	enum {
		k_chMode = k_radioChannelsStart, k_chStr, k_chThr
	};
	enum {
		k_pidStr = k_controllersStart, k_pidThr
	};
	enum {
		CH_MODE = 0, CH_STR, CH_THR
	};
	BlockPIDDfb pidStr;
	BlockPID pidThr;
public:
	CarController(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				_group(k_cntrl, PSTR("CNTRL_")),
				_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("MODE")),
				pidStr(new AP_Var_group(k_pidStr, PSTR("STR_")), 1, steeringP,
						steeringI, steeringD, steeringIMax, steeringYMax),
				pidThr(new AP_Var_group(k_pidThr, PSTR("THR_")), 1, throttleP,
						throttleI, throttleD, throttleIMax, throttleYMax,
						throttleDFCut) {
		_hal->debug->println_P(PSTR("initializing car controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 1100,
						1500, 1900));
		_hal->rc.push_back(
				new AP_RcChannel(k_chStr, PSTR("STR_"), APM_RC, 0, 1100, 1540,
						1900));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 1, 1100, 1500,
						1900));
	}
	virtual MAV_MODE getMode() {
		return (MAV_MODE) _mode.get();
	}
	virtual void update(const float & dt) {

		// check for heartbeat
		if (_hal->heartBeatLost()) {
			_mode = MAV_MODE_FAILSAFE;
			setAllRadioChannelsToNeutral();
			_hal->setState(MAV_STATE_EMERGENCY);
			_hal->debug->printf_P(PSTR("comm lost, send heartbeat from gcs\n"));
			return;
		} else if (_hal->rc[CH_THR]->getPosition() < 0.05) {
			// if throttle less than 5% cut motor power
			_mode = MAV_MODE_LOCKED;
			setAllRadioChannelsToNeutral();
			_hal->setState(MAV_STATE_STANDBY);
			return;
		} else if (_hal->getMode() == MODE_LIVE) {
			_hal->setState(MAV_STATE_ACTIVE);
		} else if (_hal->getMode() == MODE_HIL_CNTL) {
			_hal->setState(MAV_STATE_HILSIM);
		}

		// read switch to set mode
		_hal->rc[CH_MODE]->setPwm(_hal->rc[CH_MODE]->readRadio());
		if (_hal->rc[CH_MODE]->getPosition() > 0) {
			_mode = MAV_MODE_MANUAL;
		} else {
			_mode = MAV_MODE_AUTO;
		}

		// manual mode
		switch (_mode) {

		case MAV_MODE_MANUAL: {
			setAllRadioChannelsManually();
			//_hal->debug->println("manual");
			break;
		}
		case MAV_MODE_AUTO: {
			float headingError = _guide->getHeadingCommand()
					- _nav->getHeading();
			if (headingError > 180 * deg2Rad)
				headingError -= 360 * deg2Rad;
			if (headingError < -180 * deg2Rad)
				headingError += 360 * deg2Rad;
			_hal->rc[CH_STR]->setPosition(
					pidStr.update(headingError, _nav->getYawRate(), dt));
			_hal->rc[CH_THR]->setPosition(
					pidThr.update(
							_guide->getGroundSpeedCommand()
									- _nav->getGroundSpeed(), dt));
			//_hal->debug->println("automode");
			break;
		}

		default: {
			setAllRadioChannelsToNeutral();
			_mode = MAV_MODE_FAILSAFE;
			_hal->setState(MAV_STATE_EMERGENCY);
			_hal->debug->printf_P(PSTR("unknown controller mode\n"));
			break;
		}

		}
	}
};

} // namespace apo

#endif /* CONTROLLERS_H_ */
