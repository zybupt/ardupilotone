/*
 * controllers.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

namespace apo {

class BoatController: public AP_Controller {
private:
	AP_Var_group _group;
	AP_Uint8 _mode;
	enum {
		CH_MODE = 0, CH_STR, CH_THR
	};
	BlockPIDDfb pidStr;
	BlockPID pidThr;
public:
	BoatController(AP_Var::Key cntrlKey, AP_Var::Key pidStrKey,
			AP_Var::Key pidThrKey, AP_Navigator * nav, AP_Guide * guide,
			AP_HardwareAbstractionLayer * hal) :
				AP_Controller(nav, guide, hal),
				_group(cntrlKey, PSTR("CNTRL_")),
				_mode(&_group, 1, MAV_MODE_UNINIT, PSTR("MODE")),
				pidStr(new AP_Var_group(pidStrKey, PSTR("STR_")), 1, steeringP,
						steeringI, steeringD, steeringIMax, steeringYMax),
				pidThr(new AP_Var_group(pidThrKey, PSTR("THR_")), 1, throttleP,
						throttleI, throttleD, throttleIMax, throttleYMax,
						throttleDFCut) {
		_hal->debug->println_P(PSTR("initializing boat controller"));

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
			_hal->rc[CH_STR]->setUsingRadio();
			_hal->rc[CH_THR]->setUsingRadio();
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
// vim:ts=4:sw=4:expandtab
