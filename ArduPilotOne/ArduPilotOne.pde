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
#include "AP_HardwareAbstractionLayer.h"
#include "AP_RcChannelSimple.h"
#include "AP_Controller.h"
#include "AP_Navigator.h"
#include "AP_Guide.h"
#include "AP_CommLink.h"
#include "ArduPilotOne.h"
#include "controllers.h"
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

ArduPilotOne::ArduPilotOne(AP_Navigator * navigator, AP_Guide * guide, AP_Controller * controller,
		AP_HardwareAbstractionLayer * hal) :
	Loop(loop0Rate, callback0, this),
			_navigator(navigator), _guide(guide), _controller(controller), _hal(hal) {

	/*
	 * Attach loops
	 */
	Serial.println("attaching loops");
	subLoops().push_back(new Loop(loop1Rate, callback1, this));
	subLoops().push_back(new Loop(loop2Rate, callback2, this));
	subLoops().push_back(new Loop(loop3Rate, callback3, this));
	subLoops().push_back(new Loop(loop4Rate, callback4, this));

}

void ArduPilotOne::callback0(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;
	//apo->hal()->debug->println_P(PSTR("callback 0"));

	/*
	 * ahrs update
	 */
	if (apo->navigator())
		apo->navigator()->update(1.0/loop0Rate);
}

void ArduPilotOne::callback1(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;
	//apo->hal()->debug->println_P(PSTR("callback 1"));

	/*
	 * read compass
	 */

	if (apo->hal()->mode()==MODE_LIVE && apo->hal()->compass)
		apo->hal()->compass->read();

	/*
	 * hardware in the loop
	 */
	if (apo->hal()->mode()!=MODE_LIVE)
	{
		// receive message
		apo->hal()->hil->receive();

		// send messages
		apo->hal()->hil->sendMessage(MAVLINK_MSG_ID_GLOBAL_POSITION);
		apo->hal()->hil->sendMessage(MAVLINK_MSG_ID_ATTITUDE);
		apo->hal()->hil->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
	}

	/*
     * update control laws
	 */
	if (apo->guide())
		apo->guide()->update();

	/*
	 * update control laws
	 */
	if (apo->controller())
	{
		//apo->hal()->debug->println_P(PSTR("updating controller"));
		apo->controller()->update(1./loop1Rate);
	}
	/*
	char msg[50];
	sprintf(msg, "c_hdg: %f, c_thr: %f", apo->guide()->headingCommand, apo->guide()->groundSpeedCommand);
	apo->hal()->gcs->sendText(AP_CommLink::SEVERITY_LOW, msg);
	*/
	//apo->controller()->update(1. / apo->subLoops()[1]->dt());
}

void ArduPilotOne::callback2(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;
	//apo->hal()->debug->println_P(PSTR("callback 2"));

	/*
	 * compass correction for ahrs
	 */
	if (apo->hal()->compass)
	{
		apo->hal()->debug->printf_P(PSTR("compass\n"));
		apo->hal()->compass->calculate(apo->navigator()->roll,
				apo->navigator()->pitch);
	}

	/*
	 * read gps and correct position
	 */
	if (apo->hal()->gps) {
		apo->hal()->debug->printf_P(PSTR("gps\n"));
		apo->hal()->gps->update();

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
	if (apo->hal()->gcs) {
		// send messages
		apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_GLOBAL_POSITION);
		apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_ATTITUDE);
		apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
		apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_RC_CHANNELS_RAW);

		// send messages
		apo->hal()->gcs->requestCmds();
		apo->hal()->gcs->sendParameters();

		// receive messages
		apo->hal()->gcs->receive();
	}

	/*
	 * navigator debug
	 */
	 if (apo->navigator()) {
		 apo->hal()->debug->printf_P(PSTR("roll: %f pitch: %f yaw: %f\t"),
				 apo->navigator()->roll*rad2deg,
				 apo->navigator()->pitch*rad2deg,
				 apo->navigator()->yaw*rad2deg);
	 }
	 apo->hal()->debug->println();
}

void ArduPilotOne::callback3(void * data) {
	ArduPilotOne * apo = (ArduPilotOne *) data;
	//apo->hal()->debug->println_P(PSTR("callback 3"));

	/*
	 * send heartbeat
	 */
	apo->hal()->gcs->sendMessage(MAVLINK_MSG_ID_HEARTBEAT);

	/*
	 * load/loop rate/ram debug
	 */
	apo->hal()->debug->printf_P(PSTR("load: %d%%\trate: %f Hz\tfree ram: %d bytes\n"),
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
	//apo->hal()->debug->println_P(PSTR("callback 4"));
}

} // namespace apo

void setup() {

	using namespace apo;

	AP_HardwareAbstractionLayer * hal = new AP_HardwareAbstractionLayer(halMode,board,vehicle);

	/*
	 * Communications
	 */
	Serial.begin(57600, 128, 128); // debug
	Serial3.begin(57600, 128, 128); // gcs

	hal->debug = &Serial;
	hal->debug->println_P(PSTR("initializing debug line"));
	hal->debug->println_P(PSTR("initializing radio"));
	APM_RC.Init(); // APM Radio initialization

	/*
	 * Pins
	 */
	hal->debug->println_P(PSTR("settings pin modes"));
	pinMode(A_LED_PIN, OUTPUT); //  extra led
	pinMode(B_LED_PIN, OUTPUT); //  imu ledclass AP_CommLink;
	pinMode(C_LED_PIN, OUTPUT); //  gps led
	pinMode(SLIDE_SWITCH_PIN, INPUT);
	pinMode(PUSHBUTTON_PIN, INPUT);
	DDRL |= B00000100; // set port L, pint 2 to output for the relay

	/*
	 * Sensor initialization
	 */
	if (hal->mode()==MODE_LIVE)
	{
		hal->debug->println_P(PSTR("initializing adc"));
		hal->adc =  new AP_ADC_ADS7844;
		hal->adc->Init();

		hal->debug->println_P(PSTR("initializing gps"));
		hal->gps = new AP_GPS_MTK(&Serial1);

		hal->debug->println_P(PSTR("initializing baro"));
		hal->baro = new APM_BMP085_Class;
		hal->baro->Init();

		hal->debug->println_P(PSTR("initializing compass"));
		hal->compass = new AP_Compass_HMC5843;
		hal->compass->init();

		/**
		 * Initialize ultrasonic sensors. If sensors are not plugged in, the navigator will not
		 * initialize them and NULL will be assigned to those corresponding pointers.
		 * On detecting NULL assigned to any ultrasonic sensor, its corresponding block of code
		 * will not be executed by the navigator.
		 * The coordinate system is assigned by the right hand screw rule with the thumb pointing down.
		 * In set_orientation, it is defind as (front/back,left/right,down,up)
		 */

		/*
		hal->debug->println_P(PSTR("initializing front range finder"));
		hal->rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
		hal->rangeFinders[0]->init(0);
		hal->rangeFinders[0]->set_orientation(1,0,0);

		hal->debug->println_P(PSTR("initializing back range finder"));
		hal->rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
		hal->rangeFinders[1]->init(1);
		hal->rangeFinders[1]->set_orientation(-1,0,0);

		hal->debug->println_P(PSTR("initializing right range finder"));
		hal->rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
		hal->rangeFinders[2]->init(2);
		hal->rangeFinders[2]->set_orientation(0,1,0);

		hal->debug->println_P(PSTR("initializing left range finder"));
		hal->rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
		hal->rangeFinders[3]->init(3);
		hal->rangeFinders[3]->set_orientation(0,-1,0);

		hal->debug->println_P(PSTR("initializing down range finder"));
		hal->rangeFinders.push_back(new AP_RangeFinder_MaxsonarLV);
		hal->rangeFinders[4]->init(4);
		hal->rangeFinders[4]->set_orientation(0,0,1);
		*/
	}

	/*
	 * Navigator
	 */
	AP_Navigator * navigator = new DcmNavigator(hal);

	/*
	 * Guide
	 */
	AP_Guide * guide = new MavlinkGuide(navigator,hal);

	/*
	 * Controller Initialization
	 */
	AP_Controller * controller = NULL;
	switch(vehicle)
	{
	case VEHICLE_CAR:
		controller = new CarController(k_cntrl,k_pidStr,k_pidThr,navigator,guide,hal);
		break;
	case VEHICLE_QUAD:
		controller = new QuadController(navigator,guide,hal);
		break;
	}

	/*
	 * Initialize Comm Channels
	 */
	hal->debug->println_P(PSTR("initializing comm channels"));
	hal->gcs = new MavlinkComm(&Serial3,navigator,guide,controller,hal);
	if (hal->mode()==MODE_LIVE) {
		Serial1.begin(38400, 128, 16); // gps
	} else { // hil
		Serial1.begin(57600, 128, 128);
		hal->hil = new MavlinkComm(&Serial1,navigator,guide,controller,hal);
	}
	Serial.println("debug line");
	Serial1.println("gps/hil line");
	Serial3.println("gcs line");

	/*
	 * Start the autopilot
	 */
	Serial.println_P(PSTR("initializing ArduPilotOne"));
	Serial.printf_P(PSTR("free ram: %d bytes\n"),freeMemory());
	apoGlobal = new apo::ArduPilotOne(navigator,guide,controller,hal);
	hal->debug->println_P(PSTR("initialization complete"));
}

void loop() {
	apoGlobal->update();
}
