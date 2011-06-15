/*
 * AP_HardwareAbstractionLayer.h
 *
 *  Created on: Apr 4, 2011
 *
 */

#ifndef AP_HARDWAREABSTRACTIONLAYER_H_
#define AP_HARDWAREABSTRACTIONLAYER_H_

#include "../AP_Common/AP_Common.h"
#include "../FastSerial/FastSerial.h"
#include "../AP_Common/AP_Vector.h"
#include "../GCS_MAVLink/GCS_MAVLink.h"

#include "../AP_ADC/AP_ADC.h"
#include "../AP_IMU/AP_IMU.h"
#include "../AP_GPS/GPS.h"
#include "../APM_BMP085/APM_BMP085.h"
#include "../AP_Compass/AP_Compass.h"
#include "AP_RcChannel.h"
#include "../AP_RangeFinder/AP_RangeFinder.h"
#include "../GCS_MAVLink/GCS_MAVLink.h"

class AP_ADC;
class IMU;
class GPS;
class APM_BMP085_Class;
class Compass;
class BetterStream;
class RangeFinder;

namespace apo {

class AP_RcChannel;
class AP_CommLink;

// enumerations
enum halMode_t {MODE_LIVE, MODE_HIL_CNTL, /*MODE_HIL_NAV*/};
enum board_t {BOARD_ARDUPILOTMEGA};
enum vehicle_t {VEHICLE_CAR, VEHICLE_QUAD, VEHICLE_PLANE, VEHICLE_BOAT};

class AP_HardwareAbstractionLayer {

public:

	AP_HardwareAbstractionLayer(halMode_t mode, board_t board,
			vehicle_t vehicle, uint8_t heartBeatTimeout) :
		adc(), gps(), baro(), compass(), rangeFinders(), imu(), rc(), gcs(),
				hil(), debug(), load(), lastHeartBeat(),
				_heartBeatTimeout(heartBeatTimeout), _mode(mode),
				_board(board), _vehicle(vehicle), _state(MAV_STATE_UNINIT) {
	}

	/**
	 * Sensors
	 */
	AP_ADC * adc;
	GPS * gps;
	APM_BMP085_Class * baro;
	Compass * compass;
	Vector<RangeFinder *> rangeFinders;
	IMU * imu;

	/**
	 * Radio Channels
	 */
	Vector<AP_RcChannel *> rc;

	/**
	 * Communication Channels
	 */
	AP_CommLink * gcs;
	AP_CommLink * hil;
	FastSerial * debug;

	uint8_t load;
	uint32_t lastHeartBeat;

	// accessors
	halMode_t getMode() { return _mode; }
	board_t getBoard() { return _board; }
	vehicle_t getVehicle() { return _vehicle; }
	MAV_STATE getState(){ return _state; }
	void setState(MAV_STATE state) { _state = state; }

	bool heartBeatLost() {
		return ((micros() - lastHeartBeat)/1e6) > _heartBeatTimeout;
	}

private:

	// enumerations
	uint8_t _heartBeatTimeout;
	halMode_t _mode;
	board_t _board;
	vehicle_t _vehicle;
	MAV_STATE _state;
};

}

#endif /* AP_HARDWAREABSTRACTIONLAYER_H_ */
