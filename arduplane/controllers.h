/*
 * controllers.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef CONTROLLERS_H_
#define CONTROLLERS_H_

namespace apo {

//class PlaneController: public AP_Controller {
//public:
//
//	/**
//	 * note that these are not the controller radio channel numbers, they are just
//	 * unique keys so they can be reaccessed from the hal rc vector
//	 */
//	enum autoChannel_t {
//		CH_MODE = 0,
//		CH_AILERON, // this enum must match this
//		CH_ELEVATOR,
//		CH_RUDDER,
//		CH_THROTTLE,
//	};
//
//	PlaneController(AP_Navigator * nav, AP_Guide * guide,
//			AP_HardwareAbstractionLayer * hal) :
//				AP_Controller(nav, guide, hal),
////				PidDFB pidHdng2Bank, pidBank2Aileron;
////					Pid pidAlt2Thr, pidSpeed2Elevator,  pidYawRate2Rudder;
//				pidHdng2Bank(k_pidHdng2Bank, PSTR("BANK_"),heading2BankP,
//						heading2BankI, heading2BankD, heading2BankIMax,
//						heading2BankYMax),
//				pidBank2Aileron(k_pidBank2Aileron, PSTR("AIL_"), bank2AileronP,
//						bank2AileronI, bank2AileronD, bank2AileronIMax,
//						bank2AileronYMax),
//				pidPitch(k_pidPitch, PSTR("PITCH_"), PID_ATT_P, PID_ATT_I,
//						PID_ATT_D, PID_ATT_AWU, PID_ATT_LIM),
//
//				pidYaw(k_pidYaw, PSTR("YAW_"), PID_YAWPOS_P, PID_YAWPOS_I,
//						PID_YAWPOS_D, PID_YAWPOS_AWU, PID_YAWPOS_LIM),
//				pidYawRate(k_pidYawRate, PSTR("YAWRATE_"), PID_YAWSPEED_P,
//						PID_YAWSPEED_I, PID_YAWSPEED_D, PID_YAWSPEED_AWU,
//						PID_YAWSPEED_LIM, PID_YAWSPEED_DFCUT),
//				pidPN(k_pidPN, PSTR("NORTH_"), PID_POS_P,
//						PID_POS_I, PID_POS_D, PID_POS_AWU, PID_POS_LIM),
//				pidPE(k_pidPE, PSTR("EAST_"), PID_POS_P, PID_POS_I,
//						PID_POS_D, PID_POS_AWU, PID_POS_LIM),
//				pidPD(k_pidPD, PSTR("DOWN_"), PID_POS_Z_P, PID_POS_Z_I,
//						PID_POS_Z_D, PID_POS_Z_AWU, PID_POS_Z_LIM)
//	{
//		/*
//		 * allocate radio channels
//		 * the order of the channels has to match the enumeration above
//		 */
//		_hal->rc.push_back(
//		new AP_RcChannel(k_chMode, PSTR("MODE_"), APM_RC, 7, 1100, 1500, 1900, RC_MODE_IN));
//		_hal->rc.push_back(
//				new AP_RcChannel(k_chLeft, PSTR("LEFT_"), APM_RC, 0, 1100, 1100, 1900, RC_MODE_OUT));
//		_hal->rc.push_back(
//				new AP_RcChannel(k_chRight, PSTR("RIGHT_"), APM_RC, 1, 1100, 1100, 1900, RC_MODE_OUT));
//		_hal->rc.push_back(
//				new AP_RcChannel(k_chFront, PSTR("FRONT_"), APM_RC, 2, 1100, 1100, 1900, RC_MODE_OUT));
//		_hal->rc.push_back(
//				new AP_RcChannel(k_chBack, PSTR("BACK_"), APM_RC, 3, 1100, 1100, 1900, RC_MODE_OUT));
//		_hal->rc.push_back(
//				new AP_RcChannel(k_chRoll, PSTR("ROLL_"), APM_RC, 0, 1100, 1500, 1900, RC_MODE_IN));
//		_hal->rc.push_back(
//				new AP_RcChannel(k_chPitch, PSTR("PITCH_"), APM_RC, 1, 1100, 1500, 1900, RC_MODE_IN));
//		_hal->rc.push_back(
//				new AP_RcChannel(k_chYaw, PSTR("YAW_"), APM_RC, 2, 1100, 1500, 1900, RC_MODE_IN));
//		_hal->rc.push_back( // -1 -> 0 maps to 1200, linear 0-1 -> 1200-1800
//				new AP_RcChannel(k_chThr, PSTR("THRUST_"), APM_RC, 3, 1100, 1100, 1900, RC_MODE_IN));
//	}
//
//	virtual void update(const float & dt) {
//
//			if (commLost()) {
//				_mode = MAV_MODE_FAILSAFE;
//				_hal->rc[CH_LEFT]->setPosition(0);
//				_hal->rc[CH_RIGHT]->setPosition(0);
//				_hal->rc[CH_FRONT]->setPosition(0);
//				_hal->rc[CH_BACK]->setPosition(0);
//				return;
//			}
//
//			// read and set pwm so they can be read as positions later
//			_hal->rc[CH_MODE]->setPwm(_hal->rc[CH_MODE]->readRadio());
//			_hal->rc[CH_ROLL]->setPwm(_hal->rc[CH_ROLL]->readRadio());
//			_hal->rc[CH_PITCH]->setPwm(_hal->rc[CH_PITCH]->readRadio());
//			_hal->rc[CH_YAW]->setPwm(_hal->rc[CH_YAW]->readRadio());
//			_hal->rc[CH_THRUST]->setPwm(_hal->rc[CH_THRUST]->readRadio());
//
//			// manual mode
//			float mixRemoteWeight = 0;
//			if (_hal->rc[CH_MODE]->getPwm() > 1350) mixRemoteWeight = 1;
//
//			// "mix manual"
//			float cmdRoll = _hal->rc[CH_ROLL]->getPosition() * mixRemoteWeight;
//			float cmdPitch = _hal->rc[CH_PITCH]->getPosition() * mixRemoteWeight;
//			float cmdYawRate = _hal->rc[CH_YAW]->getPosition() * mixRemoteWeight;
//			float thrustMix = _hal->rc[CH_THRUST]->getPosition() * mixRemoteWeight;
//
//			// position loop
//			/*
//			float cmdNorthTilt = pidPN.update(_nav->getPN(),_nav->getVN(),dt);
//			float cmdEastTilt = pidPE.update(_nav->getPE(),_nav->getVE(),dt);
//			float cmdDown = pidPD.update(_nav->getPD(),_nav->getVD(),dt);
//
//			// "transform-to-body"
//			{
//				float trigSin = sin(-yaw);
//				float trigCos = cos(-yaw);
//				_cmdPitch = _cmdEastTilt * trigCos
//						- _cmdNorthTilt * trigSin;
//				_cmdRoll = -_cmdEastTilt * trigSin
//						+ _cmdNorthTilt * trigCos;
//				// note that the north tilt is negative of the pitch
//			}
//
//			//thrustMix += THRUST_HOVER_OFFSET;
//
//			// "thrust-trim-adjust"
//			if (fabs(_cmdRoll) > 0.5) {
//				_thrustMix *= 1.13949393;
//			} else {
//				_thrustMix /= cos(_cmdRoll);
//			}
//			if (fabs(_cmdPitch) > 0.5) {
//				_thrustMix *= 1.13949393;
//			} else {
//				_thrustMix /= cos(_cmdPitch);
//			}
//			*/
//
//			// attitude loop
//			float rollMix = pidRoll.update(cmdRoll - _nav->getRoll(),_nav->getRollRate(),dt);
//			float pitchMix = pidPitch.update(cmdPitch - _nav->getPitch(),_nav->getPitchRate(),dt);
//			float yawMix = pidYawRate.update(cmdYawRate - _nav->getYawRate(),dt);
//
//			_hal->rc[CH_LEFT]->setPosition(thrustMix + rollMix + yawMix);
//			_hal->rc[CH_RIGHT]->setPosition(thrustMix - rollMix + yawMix);
//			_hal->rc[CH_FRONT]->setPosition(thrustMix + pitchMix - yawMix);
//			_hal->rc[CH_BACK]->setPosition(thrustMix - pitchMix - yawMix);
//
//			_hal->debug->printf("L: %f\t R: %f\t F: %f\t B: %f\n",
//					_hal->rc[CH_LEFT]->getPosition(),
//					_hal->rc[CH_RIGHT]->getPosition(),
//					_hal->rc[CH_FRONT]->getPosition(),
//					_hal->rc[CH_BACK]->getPosition());
//
//			_hal->debug->printf("rollMix: %f\t pitchMix: %f\t yawMix: %f\t thrustMix: %f\n",
//								rollMix,
//								pitchMix,
//								yawMix,
//								thrustMix);
//
////			_hal->debug->printf("thrust pwm: %d\n",_hal->rc[CH_THRUST]->readRadio());
//		}
//
//private:
//	PidDFB pidHdng2Bank, pidBank2Aileron;
//	Pid pidAlt2Thr, pidSpeed2Elevator,  pidYawRate2Rudder;
//};

} // namespace apo

#endif /* CONTROLLERS_H_ */
// vim:ts=4:sw=4:expandtab
