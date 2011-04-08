/*
 * AP_Navigator.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AP_Navigator_H
#define AP_Navigator_H

#include "AP_HardwareAbstractionLayer.h"
#include "AP_IMU.h"
#include "AP_MavlinkCommand.h"
#include "constants.h"

namespace apo {

/// Navigator class
class AP_Navigator {
public:
	AP_Navigator(AP_HardwareAbstractionLayer * hal) :
		_hal(hal), _timeStamp(0), _roll(0), _rollRate(0), _pitch(0),
		_pitchRate(0), _yaw(0), _yawRate(0), _airSpeed(0), _groundSpeed(0),
		_vN(0), _vE(0), _vD(0), _pN(0), _pE(0), _pD(0), _lat_degInt(0),
		_lon_degInt(0), _alt_intM(0)
	{
	}
	virtual void calibrate() = 0;
	virtual void update(float dt) = 0;
    float getD() const
    {
        return _pD;
    }

    float getPE() const
    {
        return _pE;
    }

    float getPN() const
    {
        return _pN;
    }

    void setPD(float _pD)
    {
        this->_pD = _pD;
    }

    void setPE(float _pE)
    {
        this->_pE = _pE;
    }

    void setPN(float _pN)
    {
        this->_pN = _pN;
    }

    float getAirSpeed() const
    {
        return _airSpeed;
    }

    int32_t getAlt_intM() const
    {
        return _alt_intM;
    }

    int32_t getAlt() const
    {
        return _alt_intM / scale_m;
    }

    void setAlt(float _alt)
    {
        this->_alt_intM = _alt * scale_m;
    }

    float getLat() const
    {
        return _lat_degInt * degInt2Rad;
    }

    void setLat(float _lat)
    {
        this->_lat_degInt = _lat * rad2DegInt;
    }

    float getLon() const
    {
        return _lon_degInt * degInt2Rad;
    }

    void setLon(float _lon)
    {
        this->_lon_degInt = _lon * rad2DegInt;
    }

    float getVD() const
    {
        return _vD;
    }

    float getVE() const
    {
        return _vE;
    }

    float getGroundSpeed() const
    {
        return _groundSpeed;
    }

    int32_t getLat_degInt() const
    {
        return _lat_degInt;
    }

    int32_t getLon_degInt() const
    {
        return _lon_degInt;
    }

    float getVN() const
    {
        return _vN;
    }

    float getPitch() const
    {
        return _pitch;
    }

    float getPitchRate() const
    {
        return _pitchRate;
    }

    float getRoll() const
    {
        return _roll;
    }

    float getRollRate() const
    {
        return _rollRate;
    }

    float getYaw() const
    {
        return _yaw;
    }

    float getYawRate() const
    {
        return _yawRate;
    }

    void setAirSpeed(float _airSpeed)
    {
        this->_airSpeed = _airSpeed;
    }

    void setAlt_intM(int32_t _alt_intM)
    {
        this->_alt_intM = _alt_intM;
    }

    void setVD(float _vD)
    {
        this->_vD = _vD;
    }

    void setVE(float _vE)
    {
        this->_vE = _vE;
    }

    void setGroundSpeed(float _groundSpeed)
    {
        this->_groundSpeed = _groundSpeed;
    }

    void setLat_degInt(int32_t _lat_degInt)
    {
        this->_lat_degInt = _lat_degInt;
    }

    void setLon_degInt(int32_t _lon_degInt)
    {
        this->_lon_degInt = _lon_degInt;
    }

    void setVN(float _vN)
    {
        this->_vN = _vN;
    }

    void setPitch(float _pitch)
    {
        this->_pitch = _pitch;
    }

    void setPitchRate(float _pitchRate)
    {
        this->_pitchRate = _pitchRate;
    }

    void setRoll(float _roll)
    {
        this->_roll = _roll;
    }

    void setRollRate(float _rollRate)
    {
        this->_rollRate = _rollRate;
    }

    void setYaw(float _yaw)
    {
        this->_yaw = _yaw;
    }

    void setYawRate(float _yawRate)
    {
        this->_yawRate = _yawRate;
    }
    void setTimeStamp(int32_t _timeStamp)
    {
        this->_timeStamp = _timeStamp;
    }
    int32_t getTimeStamp() const
    {
        return _timeStamp;
    }

protected:
	AP_HardwareAbstractionLayer * _hal;
private:
	int32_t _timeStamp; // micros clock
	float _roll; // rad
	float _rollRate; //rad/s
	float _pitch; // rad
	float _pitchRate; // rad/s
	float _yaw; // rad
	float _yawRate; // rad/s
	float _airSpeed; // m/s
	float _groundSpeed; // m/s
	float _vN; // m/s
	float _vE; // m/s
	float _vD; // m/s
	float _pN; // m from home
	float _pE; // m from home
	float _pD; // m from home
	int32_t _lat_degInt; // deg / 1e7
	int32_t _lon_degInt; // deg / 1e7
	int32_t _alt_intM; // meters / 1e3
};

class DcmNavigator: public AP_Navigator {
private:
	/**
	 * Sensors
	 */

	RangeFinder * _rangeFinderDown;
	AP_DCM * _dcm;
	IMU * _imu;
	uint16_t _imuOffsetAddress;

public:
	DcmNavigator(AP_HardwareAbstractionLayer * hal) :
		AP_Navigator(hal), _dcm(), _imuOffsetAddress(0) {

		// if orientation equal to front, store as front
		/**
		 * rangeFinder<direction> is assigned values based on orientation which
		 * is specified in ArduPilotOne.pde.
		 */
		for (int i = 0; i < _hal-> rangeFinders.getSize(); i++) {
			if (_hal->rangeFinders[i] == NULL)
				continue;
			if (_hal->rangeFinders[i]->orientation_x == 0
					&& _hal->rangeFinders[i]->orientation_y == 0
					&& _hal->rangeFinders[i]->orientation_z == 1)
				_rangeFinderDown = _hal->rangeFinders[i];
		}

		if (_hal->mode() == MODE_LIVE) {
			if (_hal->adc)
				_hal->imu = new AP_IMU_Oilpan(_hal->adc, _imuOffsetAddress);
			if (_hal->imu && _hal->gps && _hal->compass)
				_dcm = new AP_DCM(_hal->imu, _hal->gps, _hal->compass);
		}
		calibrate();
	}
	virtual void calibrate() {
		// TODO: handle cold restart
		if (_hal->imu) {
			/*
			 * Gyro has built in warm up cycle and should
			 * run first */
			_hal->imu->init_gyro();
			_hal->imu->init_accel();
		}

	}
	virtual void update(float dt) {
		if (_hal->mode() != MODE_LIVE)
			return;

		setTimeStamp(micros()); // if running in live mode, record new time stamp

		/**The altitued is read off the barometer by implementing the following formula:
		 * altitude (in m) = 44330*(1-(p/po)^(1/5.255)),
		 * where, po is pressure in Pa at sea level (101325 Pa).
		 *See http://www.sparkfun.com/tutorials/253 or type this formula
		 *in a search engine for more information.
		 *altInt contains the altitude in meters.
		 */
		_hal->debug->println_P(PSTR("nav loop"));
		if (_hal->baro) {

			if (_rangeFinderDown != NULL && _rangeFinderDown->distance <= 695)
				setAlt(_rangeFinderDown->distance);

			else {
				float tmp = (_hal->baro->Press / 101325.0);
				tmp = pow(tmp, 0.190295);
				setAlt(44330 * (1.0 - tmp)); //sets the altitude in meters
			}
		}

		if (_dcm) {
			_dcm->update_DCM(dt);
			setRoll(_dcm->roll);
			setPitch(_dcm->pitch);
			setYaw(_dcm->yaw);
			setRoll(_dcm->get_gyro().x);
			setPitchRate(_dcm->get_gyro().y);
			setYawRate(_dcm->get_gyro().z);

			if (_hal->gps) {
				Matrix3f rot = _dcm->get_dcm_matrix(); // neglecting angle of attack for now
				setVN(_hal->gps->ground_speed * rot.b.x);
				setVE(_hal->gps->ground_speed * rot.b.y);
				setVE(_hal->gps->ground_speed * rot.b.z);
			}


			// need to use lat/lon and convert

			AP_MavlinkCommand home(0);
			setPN((getLat() - home.getLat())/rEarth);
			setPE((getLon() - home.getLon())*cos(home.getLat())/rEarth);
			setPD(-(getAlt() - home.getAlt()));

			/*
			 * accel/gyro debug
			 */
			/*
			 Vector3f accel = _hal->imu()->get_accel();
			 Vector3f gyro = _hal->imu()->get_gyro();
			 Serial.printf_P(PSTR("accel: %f %f %f gyro: %f %f %f\n"),
			 accel.x,accel.y,accel.z,gyro.x,gyro.y,gyro.z);
			 }
			 */

		}
	}
};

} // namespace apo

#endif // AP_Navigator_H
// vim:ts=4:sw=4:expandtab
