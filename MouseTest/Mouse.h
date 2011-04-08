/*
 * Mouse.h
 *
 *  Created on: Apr 7, 2011
 *      Author: vahuja
 */

#ifndef MOUSE_H
#define MOUSE_H

#define MOUSE_SERIAL Serial3
#define MOUSE_SERIAL_BAUD SERIAL3_BAUD  // shoudl be 115200
#define MOUSE_START_CHAR 'A'  // character that marks beginning of new message from mouse
#define MOUSE_END_CHAR 'Z'    // character that marks end of new message from mouse
#define MOUSE_SURFACE_QUALITY_CUT_OFF 12   // minimum acceptable surface quality
#define MOUSE_FIELD_OF_VIEW_DEG 70.0
#define MOUSE_FIELD_OF_VIEW_RAD ToRad(MOUSE_FIELD_OF_VIEW_DEG)
#define MOUSE_PIXELS 18.0
#define MOUSE_RAW_SCALER  5.0   // scaler that indicates how mouse returns when moved one pixel
#define MOUSE_MSG_LEN 5
#define PIXEL_TO_DEGREES_CONV (MOUSE_FIELD_OF_VIEW_DEG/(MOUSE_PIXELS*MOUSE_RAW_SCALER))
#define PIXEL_TO_RADIANS_CONV (MOUSE_FIELD_OF_VIEW_RAD/(MOUSE_PIXELS*MOUSE_RAW_SCALER))
#define DEGREES_TO_PIXELS_CONV 1 / PIXEL_TO_RADIANS_CONV
#define RADIANS_TO_PIXELS_CONV 1 / PIXEL_TO_RADIANS_CONV

#define MOUSE_DT 0.05     // loop time (in seconds) of mouse sensor (used for navigation)

// Eeprom addresses
#define EEPROM_MOUSE_BASE_ADDRESS 400
#define KP_MOUSE_ROLL_ADR  EEPROM_MOUSE_BASE_ADDRESS
#define KI_MOUSE_ROLL_ADR  EEPROM_MOUSE_BASE_ADDRESS + 4
#define KD_MOUSE_ROLL_ADR  EEPROM_MOUSE_BASE_ADDRESS + 8
#define KP_MOUSE_PITCH_ADR  EEPROM_MOUSE_BASE_ADDRESS + 12
#define KI_MOUSE_PITCH_ADR  EEPROM_MOUSE_BASE_ADDRESS + 16
#define KD_MOUSE_PITCH_ADR  EEPROM_MOUSE_BASE_ADDRESS + 20

// navigation PID values
float KP_MOUSE_ROLL = 0.5;            // 1 cm change in horizontal position will cause this angular change (in degrees)
float KI_MOUSE_ROLL = 0.0;
float KD_MOUSE_ROLL = 0.0;
float KP_MOUSE_PITCH = 0.5;           // 1 cm change in horizontal position will cause this angular change (in degrees)
float KI_MOUSE_PITCH = 0.0;
float KD_MOUSE_PITCH = 0.0;
float MOUSE_MAX_ANGLE = 20;          // maximum angular change mouse sensor will cause in controls

char mouse_buf[MOUSE_MSG_LEN];                              // data buffer for message received from sensor
int mouse_buf_ptr = 0;                                      // pointer to next available slot in data buffer
int mouse_x_raw = 0, mouse_y_raw = 0;                       // latest raw change (in pixels) from sensor
float mouse_filtered_x_cm, mouse_filtered_y_cm;             // filtered movement in cm
int mouse_surface_quality = 0;                              // latest surface quality measurement from sensor
int mouse_new_data = 0;                                     // 1 if new data has arrived from mouse sensor
float mouse_x_cm = 0.0, mouse_y_cm = 0.0;                   // latest calculated horizontal movement (in cm)
float mouse_prev_err_roll = 0.0, mouse_prev_err_pitch = 0.0;  // error from previous iteration (used by navigation PID)
float mouse_prev_roll = 0.0, mouse_prev_pitch = 0.0;        // roll and pitch of previous iteration (used by calculate)
float mouse_roll_I = 0.0, mouse_pitch_I = 0.0;
float command_mouse_roll = 0.0, command_mouse_pitch = 0.0;  // output from navigation PID

#define MOUSE_NUM_AVERAGING 3
float mouse_hist_x_cm[MOUSE_NUM_AVERAGING];
float mouse_hist_y_cm[MOUSE_NUM_AVERAGING];
int mouse_hist_ptr = 0;


#endif // MOUSE_H
