#ifndef APO_Config_H
#define APO_Config_H

#include "AP_HardwareAbstractionLayer.h"
#include "mikrokopter.h"

// Serial 0: debug      /dev/ttyUSB0
// Serial 1: gps/hil    /dev/ttyUSB1
// Serial 2: gcs        /dev/ttyUSB2


// select hardware absraction mode from
// 	MODE_LIVE, actual flight
// 	TODO: IMPLEMENT --> MODE_HIL_NAV, hardware in the loop with sensors running, tests navigation system and control
// 	MODE_HIL_CNTRL, hardware in the loop with only controller running, just tests controller
apo::halMode_t halMode = apo::MODE_HIL_CNTL;

// select from, BOARD_ARDUPILOTMEGA
apo::board_t board = apo::BOARD_ARDUPILOTMEGA;

// select from, VEHICLE_CAR, VEHICLE_QUAD, VEHICLE_PLANE
apo::vehicle_t vehicle = apo::VEHICLE_QUAD;

//---------ADVANCED SECTION ----------------//

// loop rates
static const float loop0Rate = 150;
static const float loop1Rate = 20;
static const float loop2Rate = 5;
static const float loop3Rate = 1;
static const float loop4Rate = 0.1;

//---------HARDWARE CONFIG ----------------//

//Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41
#define A_LED_PIN 37 //36 = B,3637 = A,363735 = C
#define B_LED_PIN 36
#define C_LED_PIN 35
#define EEPROM_MAX_ADDR	2048

#endif
// vim:ts=4:sw=4:expandtab
