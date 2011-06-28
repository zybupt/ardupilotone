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
	AP_Var_group _group;AP_Uint8 _mode;
	enum {
		CH_MODE = 0, CH_ROLL, CH_PITCH, CH_YAW, CH_THR
	};
	enum radioChannelKeys {
		k_chMode = k_radioChannelsStart, k_chRoll, k_chPitch, k_chYaw, k_chThr
	};
	enum controllerKeys {
		k_pidBnk2Alr = k_controllersStart,
		k_pidSpd2Elv,
		k_pidYawRt2Rddr,
		k_pidHdng2Bnk,
		k_pidAlt2Thr
	};
	BlockPIDDfb pidBnk2Alr;
	BlockPID pidSpd2Elv;
	BlockPID pidYawRt2Rddr;
	BlockPID pidHdng2Bnk;
	BlockPID pidAlt2Thr;

public:
	PlaneController(AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
	AP_Controller(nav, guide, hal),
	_group(k_cntrl, PSTR("CNTRL_")),
	_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("MODE")),
	pidBnk2Alr(new AP_Var_group(k_pidBnk2Alr, PSTR("BNK2ALR_")), 1,
			pidBnk2AlrP, pidBnk2AlrI, pidBnk2AlrD, pidBnk2AlrAwu,
			pidBnk2AlrLim),
	pidSpd2Elv(new AP_Var_group(k_pidSpd2Elv, PSTR("SPD2ELV_")), 1,
			pidSpd2ElvP, pidSpd2ElvI, pidSpd2ElvD, pidSpd2ElvAwu,
			pidSpd2ElvLim, pidSpd2ElvDFCut),
	pidYawRt2Rddr(new AP_Var_group(k_pidYawRt2Rddr, PSTR("YAWRT2RDDR_")), 1,
			pidYawRt2RddrP, pidYawRt2RddrI, pidYawRt2RddrD, pidYawRt2RddrAwu,
			pidYawRt2RddrLim, pidYawRt2RddrDFCut),
	pidHdng2Bnk(new AP_Var_group(k_pidHdng2Bnk, PSTR("HDNG2BNK_")), 1,
			pidHdng2BnkP, pidHdng2BnkI, pidHdng2BnkD, pidHdng2BnkAwu,
			pidHdng2BnkLim, pidHdng2BnkDFCut),
	pidAlt2Thr(new AP_Var_group(k_pidAlt2Thr, PSTR("ALT2THR_")), 1,
			pidAlt2ThrP, pidAlt2ThrI, pidAlt2ThrD, pidAlt2ThrAwu,
			pidAlt2ThrLim, pidAlt2ThrDFCut)
	{

		_hal->debug->println_P(PSTR("initializing car controller"));

		_hal->rc.push_back(
				new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 5, 1100,
						1500, 1900, RC_MODE_IN));
		_hal->rc.push_back(
				new AP_RcChannel(k_chRoll, PSTR("ROLL_"), APM_RC, 0, 1200,
						1500, 1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chPitch, PSTR("PITCH_"), APM_RC, 1, 1200,
						1500, 1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chYaw, PSTR("YAW_"), APM_RC, 2, 1200,
						1500, 1800, RC_MODE_INOUT));
		_hal->rc.push_back(
				new AP_RcChannel(k_chThr, PSTR("THR_"), APM_RC, 3, 1100, 1100,
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
		} else if (_hal->rc[CH_THR]->getRadioPosition() < 0.05) {
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
		if (_hal->rc[CH_MODE]->getRadioPosition() > 0) {
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
			const static float desiredAirSpeed = 10; // m/s
			const static float desiredAltitude = 10; // m
			_hal->rc[CH_ROLL]->setPosition(
					pidBnk2Alr.update(pidHdng2Bnk.update(headingError, dt),
							_nav->getRollRate(), dt));
			_hal->rc[CH_PITCH]->setPosition(
								pidSpd2Elv.update(desiredAirSpeed - _nav->getAirSpeed(), dt));
			_hal->rc[CH_YAW]->setPosition(
					pidYawRt2Rddr.update(-_nav->getYawRate(), dt)); // desired is zero, needs washout
			_hal->rc[CH_THR]->setPosition(
					pidAlt2Thr.update(desiredAltitude - _nav->getAlt(), dt));
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
