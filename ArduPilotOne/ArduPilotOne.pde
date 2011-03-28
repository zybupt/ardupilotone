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

namespace apo
{

class AP_CommLink;

ArduPilotOne::ArduPilotOne(
    BetterStream & debug,
    BetterStream & gcs,
    AP_ADC * adc, GPS * gps,
    APM_BMP085_Class * baro, Compass * compass):
    Loop(LOOP_0_RATE,callback0,this),
    _debug(debug),
    _gcs(new MavlinkComm(&gcs,this)),
    _controller(NULL),
    _adc(adc), _gps(gps), _baro(baro), _compass(compass)
{

    /*
     * Attach loops
     */
    Serial.println("attaching loops");
    subLoops().push_back(new Loop(LOOP_1_RATE,callback1,this));
    subLoops().push_back(new Loop(LOOP_2_RATE,callback2,this));
    subLoops().push_back(new Loop(LOOP_3_RATE,callback3,this));
    subLoops().push_back(new Loop(LOOP_4_RATE,callback4,this));

    /*
     * Pins
     */
    Serial.println("settings pin modes");
    pinMode(A_LED_PIN, OUTPUT); //  extra led
    pinMode(B_LED_PIN, OUTPUT); //  imu led
    pinMode(C_LED_PIN, OUTPUT); //  gps led
    pinMode(SLIDE_SWITCH_PIN, INPUT);
    pinMode(PUSHBUTTON_PIN, INPUT);
    DDRL |= B00000100; // set port L, pint 2 to output for the relay

    /*
     * Component initialization
     */
    APM_RC.Init(); // APM Radio initialization

    controllerInit();
    if (_adc) {
        Serial.println("initializing adc");
        _adc->Init();
    }
    if (_gps) {
        Serial.println("initializing gps");
        _gps->init();
    }
    if (_baro) {
        Serial.println("initializing baro");
        _baro->Init();
    }
    if (_compass) {
        Serial.println("initializing compass");
        _compass->init();
    }

    /*
     * Navigator
     */
    _navigator = new DcmNavigator(AP_Navigator::MODE_LIVE,_adc,_gps,_baro,_compass);

    getDebug().println("initialization complete");
}

void ArduPilotOne::callback0(void * data)
{
    ArduPilotOne * apo = (ArduPilotOne *)data;

    /*
     * ahrs update
     */
    if (apo->navigator()) apo->navigator()->update(apo->dt());
}

void ArduPilotOne::callback1(void * data)
{
    ArduPilotOne * apo = (ArduPilotOne *)data;

    /*
     * read compass
     */
    if (apo->compass()) apo->compass()->read();

    /*
     * update control laws
     */
    if (apo->controller()) apo->controller()->update(1./apo->subLoops()[1]->dt());
}

void ArduPilotOne::callback2(void * data)
{
    ArduPilotOne * apo = (ArduPilotOne *)data;

    /*
     * compass correction for ahrs
     */
    if (apo->compass()) apo->compass()->calculate(apo->navigator()->roll,apo->navigator()->pitch);

    /*
     * read gps and correct position
     */
    if (apo->gps()) {
        apo->gps()->update();

        // debug
        apo->getDebug().printf_P(PSTR("lat: %ld lng: %ld alt: %ld\t"),
                                 apo->gps()->latitude,
                                 apo->gps()->longitude,
                                 apo->gps()->altitude);
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

        // send messages
        apo->gcs()->requestCmds();
        apo->gcs()->sendParameters();

        // receive messages
        apo->gcs()->receive();
    }

    /*
     * navigator debug
     */
    if (apo->navigator()) {
        apo->getDebug().printf_P(PSTR("roll: %f pitch: %f yaw: %f\t"),
                                 apo->navigator()->roll*rad2deg,
                                 apo->navigator()->pitch*rad2deg,
                                 apo->navigator()->yaw*rad2deg);
    }
    apo->getDebug().println();
}

void ArduPilotOne::callback3(void * data)
{
    ArduPilotOne * apo = (ArduPilotOne *)data;

    /*
     * send heartbeat
     */
    apo->gcs()->sendMessage(AP_CommLink::MSG_HEARTBEAT);

    /*
     * load debug
     */
    apo->getDebug().printf_P(PSTR("load: %d%%\n"),apo->load());

    /*
     * adc debug
     */
    //apo->getDebug().printf_P(PSTR("adc: %d %d %d %d %d %d %d %d\n"),
    //apo->adc()->Ch(0), apo->adc()->Ch(1), apo->adc()->Ch(2),
    //apo->adc()->Ch(3), apo->adc()->Ch(4), apo->adc()->Ch(5),
    //apo->adc()->Ch(6), apo->adc()->Ch(7), apo->adc()->Ch(8));

}

void ArduPilotOne::callback4(void * data)
{
    ArduPilotOne * apo = (ArduPilotOne *)data;
}

} // namespace apo

void setup()
{
    Serial.begin(115200,128,128);  // debug
    Serial1.begin(115200,128,128); // gps
    Serial3.begin(115200,128,128); // gcs

    Serial.println("starting APO");

    apoGlobal = new apo::ArduPilotOne(Serial,Serial3,
                                      new AP_ADC_ADS7844, new AP_GPS_UBLOX(&Serial1),
                                      new APM_BMP085_Class, new AP_Compass_HMC5843);
}

void loop()
{
    apoGlobal->update();
}
