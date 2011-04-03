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

namespace apo {

/// Navigator class
class AP_Navigator {
public:
	enum mode_t {
		MODE_LIVE, MODE_HIL_CNTL, MODE_HIL_NAV,
	};
	AP_Navigator(mode_t mode) :
		_mode(mode) {
	}
	virtual void calibrate() = 0;
	virtual void update(float dt) = 0;
	float roll;
	float rollRate;
	float pitch;
	float pitchRate;
	float yaw;
	float yawRate;
	float airSpeed;
	float groundSpeed;
	float vN;
	float vE;
	float vD;
	int32_t latInt;
	int32_t lonInt;
	int32_t altInt;
	float lat() {
		return latInt / 1e7;
	}
	float lon() {
		return lonInt / 1e7;
	}
	float alt() {
		return altInt / 1e3;
	}
	void setLat(float lat) {
		latInt = 1e7 * lat;
	}
	void setLon(float lon) {
		lonInt = 1e7 * lon;
	}
	void setAlt(float alt) {
		altInt = 1e3 * alt;
	}
protected:
	mode_t _mode;
};

class DcmNavigator: public AP_Navigator {
private:
	/**
	 * Sensors
	 * TODO: Abstract all sensor libraries to allow using
	 * ArduPilot and eventually newer hardware.
	 */
	AP_ADC * _adc;
	GPS * _gps;
	APM_BMP085_Class * _baro;
	Compass * _compass;
	IMU * _imu;
	AP_DCM * _dcm;
	uint16_t _imuOffsetAddress;
public:
	DcmNavigator(mode_t mode, AP_ADC * adc = NULL, GPS * gps = NULL,
			APM_BMP085_Class * baro = NULL, Compass * compass = NULL) :
		AP_Navigator(mode), _adc(adc), _gps(gps), _baro(baro),
				_compass(compass), _imu(), _dcm(), _imuOffsetAddress(0) {
		if (mode == MODE_LIVE) {
			if (_adc)
				_imu = new AP_IMU_Oilpan(_adc, _imuOffsetAddress);
			if (_imu && _gps && _compass)
				_dcm = new AP_DCM(_imu, _gps, _compass);
		}
		calibrate();
	}
	virtual void calibrate() {
		// TODO: handle cold restart
		if (_imu) {
			/*
			 * Gyro has built in warm up cycle and should
			 * run first */_imu->init_gyro();
			_imu->init_accel();
		}

	}
	virtual void update(float dt) {
		if (_mode != MODE_LIVE)
			return;

		if(_baro) {
			/**The following 4 lines implement the following formula:
			 * altitude (in m) = 44330*(1-(p/po)^(1/5.255)).
			 * Here, po is pressure in Pa at sea level (101325 Pa)
			 *See http://www.sparkfun.com/tutorials/253 or type this formula in a search engine for more information.
			 */
			float tmp = (_baro->Press / 101325.0);
			tmp = pow(tmp, 0.190295);
			setAlt(44330 * (1.0 - tmp)); //sets the altitude in meters
		}
		if (_dcm) {
			_dcm->update_DCM(dt);
			roll = _dcm->roll;
			pitch = _dcm->pitch;
			yaw = _dcm->yaw;
			rollRate = _dcm->get_gyro().x;
			pitchRate = _dcm->get_gyro().y;
			yawRate = _dcm->get_gyro().z;

			if (_gps) {
				Matrix3f rot = _dcm->get_dcm_matrix(); // neglecting angle of attack for now
				vN = _gps->ground_speed * rot.b.x;
				vE = _gps->ground_speed * rot.b.y;
				vD = _gps->ground_speed * rot.b.z;
			}

			/*
			 * accel/gyro debug
			 */
			/*
			 Vector3f accel = _imu()->get_accel();
			 Vector3f gyro = _imu()->get_gyro();
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
