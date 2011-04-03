#ifndef defines_H
#define defines_H

const float loop0Rate = 150;

#define LOOP_0_RATE 150
#define LOOP_1_RATE 20
#define LOOP_2_RATE 5
#define LOOP_3_RATE 1
#define LOOP_4_RATE 0.1

#define BOARD_ARDUPILOTMEGA 1

#define VEHICLE_CAR 1
#define VEHICLE_QUAD 2
#define VEHICLE_PLANE 3

#define RUNMODE_LIVE 1
#define RUNMODE_HIL_STATE 2

#define ENABLE 1
#define DISABLE 0

//Hardware Parameters
#define SLIDE_SWITCH_PIN 40
#define PUSHBUTTON_PIN 41

#define A_LED_PIN 37//36 = B,3637 = A,363735 = C
#define B_LED_PIN 36
#define C_LED_PIN 35

// EEPROM addresses
#define EEPROM_MAX_ADDR		4096
// parameters get the first 1KiB of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x400 // where in memory home WP is stored + all other WP
#define WP_SIZE 20 // mavlink_command_t size


#include "APO_Config.h"

#ifndef VEHICLE_TYPE
#error must define VEHICLE_TYPE
#endif

#ifndef BOARD_TYPE
#error must define BOARD_TYPE
#endif

#ifndef RUNMODE_TYPE
#error must define RUNMODE_TYPE
#endif

#endif // defines_H
// vim:ts=4:sw=4:expandtab
