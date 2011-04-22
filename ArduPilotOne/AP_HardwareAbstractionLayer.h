/*
 * AP_HardwareAbstractionLayer.h
 *
 *  Created on: Apr 4, 2011
 *
 */

#ifndef AP_HARDWAREABSTRACTIONLAYER_H_
#define AP_HARDWAREABSTRACTIONLAYER_H_

#include "AP_ADC.h"
#include "AP_IMU.h"
#include "AP_GPS.h"
#include "APM_BMP085.h"
#include "AP_Compass.h"
#include "AP_Common.h"
#include "AP_RcChannelSimple.h"
#include "AP_RangeFinder.h"
#include "AP_BatteryVoltage.h"

class AP_ADC;
class IMU;
class GPS;
class APM_BMP085;
class Compass;
class BetterStream;
class AP_RcChannelSimple;
class RangeFinder;
class Battery;

namespace apo {

class AP_CommLink;

// enumerations
enum halMode_t {MODE_LIVE, MODE_HIL_CNTL, /*MODE_HIL_NAV*/};
enum board_t {BOARD_ARDUPILOTMEGA};
enum vehicle_t {VEHICLE_CAR, VEHICLE_QUAD, VEHICLE_PLANE};

class AP_HardwareAbstractionLayer {

public:
	AP_HardwareAbstractionLayer(halMode_t mode, board_t board, vehicle_t vehicle) :
		_mode(mode), _board(board), _vehicle(vehicle), adc(),
		gps(), baro(), compass(), rangeFinders(),
		imu(), rc(), gcs(), hil(), debug(), battery()
	{
	}
	/**
	 * Sensors
	 */
	AP_BatteryVoltage * battery;
	AP_ADC * adc;
	GPS * gps;
	APM_BMP085_Class * baro;
	Compass * compass;
	Vector<RangeFinder *> rangeFinders;
	IMU * imu;

	/**
	 * Radio Channels
	 */
	Vector<AP_RcChannelSimple *> rc;

	/**
	 * Communication Channels
	 */
	AP_CommLink * gcs;
	AP_CommLink * hil;
	FastSerial * debug;

	// accessors
	halMode_t mode() { return _mode; }
	board_t board() { return _board; }
	vehicle_t vehicle() { return _vehicle; }

private:
	// enumerations
	halMode_t _mode;
	board_t _board;
	vehicle_t _vehicle;
};

}

#endif /* AP_HARDWAREABSTRACTIONLAYER_H_ */
