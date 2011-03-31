#ifndef config_H
#define config_H

#include "APO_Config.h"

// definitions
#define ARDUPILOT 1
#define ARDUPILOTMEGA 2
#define ENABLED 1
#define DISABLED 0
#define CAR 1
#define PLANE 2
#define QUAD 3

// required values
#ifndef BOARD
	#error Must define BOARD
#endif
#ifndef VEHICLE_TYPE
	#error Must define VEHICLE_TYPE
#endif
#ifndef HIL_MODE
	#error Must define HIL_MODE
#endif

// default settings
#ifndef AIRSPEED_SENSOR
	#define AIRSPEED_SENSOR DISABLED
#endif
#ifndef LOOP_0_RATE
	#define LOOP_0_RATE 150
#endif
#ifndef LOOP_1_RATE
	#define LOOP_1_RATE 20
#endif
#ifndef LOOP_2_RATE
	#define LOOP_2_RATE 5
#endif
#ifndef LOOP_3_RATE
	#define LOOP_3_RATE 1
#endif
#ifndef LOOP_4_RATE
	#define LOOP_4_RATE 0.1
#endif

//Hardware Parameters
#ifndef SLIDE_SWITCH_PIN
	#define SLIDE_SWITCH_PIN 40
#endif
#ifndef PUSHBUTTON_PIN
	#define PUSHBUTTON_PIN 41
#endif
#ifndef A_LED_PIN
	#define A_LED_PIN 37//36 = B,3637 = A,363735 = C
#endif
#ifndef B_LED_PIN
	#define B_LED_PIN 36
#endif
#ifndef C_LED_PIN
	#define C_LED_PIN 35
#endif

#endif // config_H

// vim:ts=4:sw=4:expandtab
