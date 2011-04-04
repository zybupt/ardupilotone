/*
 * AP_HardwareAbstractionLayer.h
 *
 *  Created on: Apr 4, 2011
 *      Author: ostaton
 */

#ifndef AP_HARDWAREABSTRACTIONLAYER_H_
#define AP_HARDWAREABSTRACTIONLAYER_H_

class AP_ADC;
class GPS;
class APM_BMP085;
class Compass;
class BetterStream;
class AP_RcChannel;
class RangeFinder;

namespace apo {

class AP_CommLink;

class AP_HardwareAbstractionLayer {

public:

	/**
	 * Sensors
	 */
	AP_ADC * adc;
	GPS * gps;
	APM_BMP085_Class * baro;
	Compass * compass;
	Vector<RangeFinder *> rangeFinders;

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
};

}

#endif /* AP_HARDWAREABSTRACTIONLAYER_H_ */
