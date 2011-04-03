/*
 * ArduPilotOne.pde
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
/*
 * AVR runtime
 */
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <math.h>
/*
 * Libraries
 */
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_GPS.h>
#include <APM_RC.h>
#include <AP_ADC.h>
#include <APM_BMP085.h>
#include <AP_Compass.h>
#include <AP_Math.h>
#include <Wire.h>
#include <AP_IMU.h>
#include <AP_DCM.h>
#include <AP_Loop.h>
#include <GCS_MAVLink.h>
#include <AP_RangeFinder.h>
/*
 * Local Modules
 */
#include "AP_RcChannel.h"
#include "AP_Controller.h"
#include "AP_Navigator.h"
#include "AP_Guide.h"
#include "AP_CommLink.h"
#include "ArduPilotOne.h"
/*
 * Required Global Declarations
 */
FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);
apo::ArduPilotOne * apoGlobal = NULL;

namespace apo {

class AP_CommLink;

ArduPilotOne::ArduPilotOne(BetterStream & debug, BetterStream & gcs, BetterStream & hil,
		AP_ADC * adc = NULL, GPS * gps = NULL, APM_BMP085_Class * baro = NULL,
		Compass * compass = NULL, Vector<RangeFinder*> * rangeFinders = NULL) :
	Loop(LOOP_0_RATE, callback0, this), _debug(debug),
			_gcs(new MavlinkComm(&gcs, this)), _hil(NULL), _controller(NULL), _adc(adc),
			_gps(gps), _baro(baro), _compass(compass) {

	/*
	 * Start HIL if requested
	 */
#if RUNMODE_TYPE == RUNMODE_HIL_STATE
	_hil = new MavlinkComm(&hil,this);
#endif

	/*
	 * Attach loops
	 */
	Serial.println("attaching loops");
	subLoops().push_back(new Loop(LOOP_1_RATE, callback1, this));
	subLoops().push_back(new Loop(LOOP_2_RATE, callback2, this));
	subLoops().push_back(new Loop(LOOP_3_RATE, callback3, this));
	subLoops().push_back(new Loop(LOOP_4_RATE, callback4, this));


	/*
	 * Navigator
	 */
	_navigator = new DcmNavigator(AP_Navigator::MODE_LIVE, _adc, _gps, _baro,
			_compass);

	/*
	 * Guide
	 */
	_guide = new MavlinkGuide(_navigator/*, frontRangeFinder, backRangeFinder,
	 leftRangeFinder, rightRangeFinder*/);

	/*
	 * Controller Initialization
	 */
	controllerInit();

	getDebug().println_P(PSTR("initialization complete"));

}

void ArduPilotOne::callback0(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;

	/*
	 * ahrs update
	 */
	if (apo->navigator())
		apo->navigator()->update(apo->dt());
}

void ArduPilotOne::callback1(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;

	/*
	 * read compass
	 */
	if (apo->compass())
		apo->compass()->read();

	/*
	 * hardware in the loop
	 */
#if RUNMODE_TYPE == RUNMODE_HIL_STATE
	if (apo->hil())
	{
		// receive message
		apo->hil()->receive();

		// send messages
		apo->hil()->sendMessage(AP_CommLink::MSG_LOCATION);
		apo->hil()->sendMessage(AP_CommLink::MSG_ATTITUDE);
		apo->hil()->sendMessage(AP_CommLink::MSG_SERVO_OUT);
	}
#endif
	/*
	 * update control laws
	 */
	if (apo->controller())
		apo->controller()->update(1. / apo->subLoops()[1]->dt());
}

void ArduPilotOne::callback2(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;

	/*
	 * compass correction for ahrs
	 */
	if (apo->compass())
		apo->compass()->calculate(apo->navigator()->roll,
				apo->navigator()->pitch);

	/*
	 * read gps and correct position
	 */
	if (apo->gps()) {
		apo->gps()->update();

		// debug
		/*
		 apo->getDebug().printf_P(PSTR("lat: %ld lng: %ld alt: %ld\t"),
		 apo->gps()->latitude,
		 apo->gps()->longitude,
		 apo->gps()->altitude);
		 */
	}

	/*
	 * handle ground control station communication
	 */
	{
	}
	if (apo->gcs()) {
		// send messages
		apo->gcs()->sendMessage(AP_CommLink::MSG_LOCATION);
		apo->gcs()->sendMessage(AP_CommLink::MSG_ATTITUDE);
		apo->gcs()->sendMessage(AP_CommLink::MSG_SERVO_OUT);
		apo->gcs()->sendMessage(AP_CommLink::MSG_RADIO_OUT);

		// send messages
		apo->gcs()->requestCmds();
		apo->gcs()->sendParameters();

		// receive messages
		apo->gcs()->receive();
	}

	/*
	 * navigator debug
	 */
	/*
	 if (apo->navigator()) {
	 apo->getDebug().printf_P(PSTR("roll: %f pitch: %f yaw: %f\t"),
	 apo->navigator()->roll*rad2deg,
	 apo->navigator()->pitch*rad2deg,
	 apo->navigator()->yaw*rad2deg);
	 }
	 */
	//apo->getDebug().println();
}

void ArduPilotOne::callback3(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;

	/*
	 * send heartbeat
	 */
	apo->gcs()->sendMessage(AP_CommLink::MSG_HEARTBEAT);

	/*
	 * load/loop rate/ram debug
	 */
	apo->getDebug().printf_P(PSTR("load: %d%%\trate: %f Hz\tfree ram: %d bytes\n"),
			apo->load(),1.0/apo->dt(),freeMemory());

	/*
	 * adc debug
	 */
	//apo->getDebug().printf_P(PSTR("adc: %d %d %d %d %d %d %d %d\n"),
	//apo->adc()->Ch(0), apo->adc()->Ch(1), apo->adc()->Ch(2),
	//apo->adc()->Ch(3), apo->adc()->Ch(4), apo->adc()->Ch(5),
	//apo->adc()->Ch(6), apo->adc()->Ch(7), apo->adc()->Ch(8));

}

void ArduPilotOne::callback4(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;
}

} // namespace apo

void setup() {
	Serial.begin(57600, 128, 128); // debug
	Serial1.begin(57600, 128, 128); // gps
	Serial3.begin(57600, 128, 128); // gcs

	/*
	 * Pins
	 */
	Serial.println_P(PSTR("settings pin modes"));
	pinMode(A_LED_PIN, OUTPUT); //  extra led
	pinMode(B_LED_PIN, OUTPUT); //  imu led
	pinMode(C_LED_PIN, OUTPUT); //  gps led
	pinMode(SLIDE_SWITCH_PIN, INPUT);
	pinMode(PUSHBUTTON_PIN, INPUT);
	DDRL |= B00000100; // set port L, pint 2 to output for the relay

	/*
	 * Sensor initialization
	 */
	Serial.println_P(PSTR("initializing radio"));
	APM_RC.Init(); // APM Radio initialization

	Serial.println_P(PSTR("initializing adc"));
	AP_ADC * adc = new AP_ADC_ADS7844;
	adc->Init();;

	Serial.println_P(PSTR("initializing gps"));
	GPS * gps = new AP_GPS_MTK(&Serial1);

	Serial.println_P(PSTR("initializing baro"));
	APM_BMP085_Class * baro = new APM_BMP085_Class;
	baro->Init();

	Serial.println_P(PSTR("initializing compass"));
	delay(1000);
	Compass * compass = new AP_Compass_HMC5843;
	compass->init();

	Serial.println_P(PSTR("initializing front range finder"));
	Vector<RangeFinder *> rangeFinders;
	rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
	rangeFinders[0]->init(0);
	rangeFinders[0]->set_orientation(1,0,0);

	Serial.println_P(PSTR("initializing back range finder"));
	rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
	rangeFinders[0]->init(1);
	rangeFinders[0]->set_orientation(-1,0,0);

	Serial.println_P(PSTR("initializing down range finder"));
	rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
	rangeFinders[0]->init(2);
	rangeFinders[0]->set_orientation(0,0,1);

	/*
	 * Start the autopilot
	 */
	Serial.println_P(PSTR("initializing ArduPilotOne"));
	Serial.printf_P(PSTR("free ram: %d bytes\n"),freeMemory());
	apoGlobal = new apo::ArduPilotOne(Serial, Serial3, Serial, adc,
		gps, baro, compass, &rangeFinders);

}

void loop() {
	apoGlobal->update();
}
