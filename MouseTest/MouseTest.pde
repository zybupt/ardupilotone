/*
 * MouseTest.pde
 *
 *  Created on: Apr 7, 2011
 *      Author: vahuja
 */
#include "Mouse.h"
#include "Ardu.h"
#include <EEPROM.h>
#include <DataFlash.h>


void writeEEPROM(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;

  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++)
    EEPROM.write(address + i, floatIn.floatByte[i]);
}


//
// mouse_init - initialises the serial port for communication with the mouse sensor
//

// Write a Mouse Senor data packet
void Log_Write_Mouse(int raw_x, int raw_y, int surface_quality, int expected_x, int expected_y, int x_cm, int y_cm)
{
  DataFlash.WriteByte(HEAD_BYTE1);
  DataFlash.WriteByte(HEAD_BYTE2);
  DataFlash.WriteByte(LOG_MOUSE_MSG);
  DataFlash.WriteInt(raw_x);
  DataFlash.WriteInt(raw_y);
  DataFlash.WriteInt(surface_quality);
  DataFlash.WriteInt(expected_x);
  DataFlash.WriteInt(expected_y);
  DataFlash.WriteInt(x_cm);
  DataFlash.WriteInt(y_cm);
  DataFlash.WriteByte(END_BYTE);
}


void cspc() {
  Serial.print(", ");
}



void Log_Write_PID(byte num_PID, int P, int I,int D, int output)
{
  DataFlash.WriteByte(HEAD_BYTE1);
  DataFlash.WriteByte(HEAD_BYTE2);
  DataFlash.WriteByte(LOG_PID_MSG);
  DataFlash.WriteByte(num_PID);
  DataFlash.WriteInt(P);
  DataFlash.WriteInt(I);
  DataFlash.WriteInt(D);
  DataFlash.WriteInt(output);
  DataFlash.WriteByte(END_BYTE);
}


float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (SerAva() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = SerRea();
      timeout = 0;
      index++;
    }
  }
  while ((data[constrain(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}




float readEEPROM(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;

  for (int i = 0; i < 4; i++)
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}


void mouse_init()
{
    MOUSE_SERIAL.begin(MOUSE_SERIAL_BAUD);

    // load values from EEPROM
    KP_MOUSE_ROLL = readEEPROM(KP_MOUSE_ROLL_ADR);            // 1 cm change in horizontal position will cause this angular change (in degrees)
    KI_MOUSE_ROLL = readEEPROM(KI_MOUSE_ROLL_ADR);
    KD_MOUSE_ROLL = readEEPROM(KD_MOUSE_ROLL_ADR);
    KP_MOUSE_PITCH = readEEPROM(KP_MOUSE_PITCH_ADR);              // 1 cm change in horizontal position will cause this angular change (in degrees)
    KI_MOUSE_PITCH = readEEPROM(KI_MOUSE_PITCH_ADR);
    KD_MOUSE_PITCH = readEEPROM(KD_MOUSE_PITCH_ADR);
}

//
// mouse_capture_PIDs - prompt and capture PIDs from user
//
void mouse_capture_PIDs()
{
    int saveToEeprom = 0;
    float tempVal1, tempVal2, tempVal3;

    // display current values
    SerPrln();
    SerPri("Mouse Roll PID: ");
    SerPri(KP_MOUSE_ROLL); cspc();
    SerPri(KI_MOUSE_ROLL); cspc();
    SerPrln(KD_MOUSE_ROLL);
    SerPri("Mouse Pitch PID: ");
    SerPri(KP_MOUSE_PITCH); cspc();
    SerPri(KI_MOUSE_PITCH); cspc();
    SerPrln(KD_MOUSE_PITCH);

    // MOUSE ROLL PIDs
    SerFlu();
    SerPri("Enter new Mouse Roll P;I;D; values or 0 to skip: ");
    while( !SerAva() );  // wait until user presses a key
    tempVal1 = readFloatSerial();
    tempVal2 = readFloatSerial();
    tempVal3 = readFloatSerial();
    if( tempVal1 != 0 || tempVal2 != 0 || tempVal3 != 0 ) {
        KP_MOUSE_ROLL = tempVal1;            // 1 cm change in horizontal position will cause this angular change (in degrees)
        KI_MOUSE_ROLL = tempVal2;
        KD_MOUSE_ROLL = tempVal3;
        SerPrln();
        SerPri("P:");
        SerPri(KP_MOUSE_ROLL);
        SerPri("\tI:");
        SerPri(KI_MOUSE_ROLL);
        SerPri("\tD:");
        SerPri(KD_MOUSE_ROLL);
        saveToEeprom = 1;
    }
    SerPrln();

    // MOUSE PITCH PIDs
    SerFlu();
    SerPri("Enter new Mouse Pitch P;I;D; values or 0 to skip: ");
    while( !SerAva() );  // wait until user presses a key
    tempVal1 = readFloatSerial();
    tempVal2 = readFloatSerial();
    tempVal3 = readFloatSerial();
    if( tempVal1 != 0 || tempVal2 != 0 || tempVal3 != 0 ) {
        KP_MOUSE_PITCH = tempVal1;              // 1 cm change in horizontal position will cause this angular change (in degrees)
        KI_MOUSE_PITCH = tempVal2;
        KD_MOUSE_PITCH = tempVal3;
        SerPrln();
        SerPri("P:");
        SerPri(KP_MOUSE_PITCH);
        SerPri("\tI:");
        SerPri(KI_MOUSE_PITCH);
        SerPri("\tD:");
        SerPri(KD_MOUSE_PITCH);
        saveToEeprom = 1;
    }
    SerPrln();

    if( saveToEeprom ) {
        writeEEPROM(KP_MOUSE_ROLL,KP_MOUSE_ROLL_ADR);
        writeEEPROM(KI_MOUSE_ROLL,KI_MOUSE_ROLL_ADR);
        writeEEPROM(KD_MOUSE_ROLL,KD_MOUSE_ROLL_ADR);
        writeEEPROM(KP_MOUSE_PITCH,KP_MOUSE_PITCH_ADR);
        writeEEPROM(KI_MOUSE_PITCH,KI_MOUSE_PITCH_ADR);
        writeEEPROM(KD_MOUSE_PITCH,KD_MOUSE_PITCH_ADR);
        SerPrln("written to eeprom.");
    }else{
        SerPrln("No changes, nothing written to eeprom.");
    }
}

//
// mouse_addChar - adds a character from the serial port attached to mouse
//
void mouse_addChar(char newChar)
{
    int x,y,surface_quality,i;
    // add new character to buffer
    mouse_buf[mouse_buf_ptr++] = newChar;
    mouse_buf_ptr %= MOUSE_MSG_LEN;

    // check if full message received
    if( newChar == MOUSE_END_CHAR && mouse_buf[mouse_buf_ptr] == MOUSE_START_CHAR ) {
        mouse_x_raw = (int)(mouse_buf[(mouse_buf_ptr+1) % MOUSE_MSG_LEN]);
        mouse_y_raw = (int)(mouse_buf[(mouse_buf_ptr+2) % MOUSE_MSG_LEN]);
        mouse_surface_quality = (int)((byte)(mouse_buf[(mouse_buf_ptr+3) % MOUSE_MSG_LEN]));

        // calculate horizontal movements
        mouse_calculate();

        // alert that new data has arrived
        mouse_new_data = 1;
    }
}

//
// mouse_calculate - calculate recent horizontal movements
//
void mouse_calculate()
{
    static float conv_factor = (1.0/(float)(MOUSE_PIXELS*MOUSE_RAW_SCALER))*2.0*tan((float)MOUSE_FIELD_OF_VIEW_RAD/2.0);  // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
    static float radians_to_pixels = RADIANS_TO_PIXELS_CONV;
    float diff_roll = roll - mouse_prev_roll;
    float diff_pitch = pitch - mouse_prev_pitch;
    float exp_change_x, exp_change_y;
    float change_x, change_y;
    int i;

    // if mouse surface quality is acceptable calculate move
    //if( mouse_surface_quality < 0 || mouse_surface_quality >= MOUSE_SURFACE_QUALITY_CUT_OFF ) {

        // calculate expected x,y diff due to roll and pitch change
        exp_change_x = -diff_roll * radians_to_pixels;
        exp_change_y = diff_pitch * radians_to_pixels;

        // real estimated raw change from mouse
        change_x = mouse_x_raw - exp_change_x;
        change_y = mouse_y_raw - exp_change_y;

        // convert raw change to horizontal movement in cm
        mouse_x_cm = change_x * press_sonar_altitude * conv_factor;
        mouse_y_cm = change_y * press_sonar_altitude * conv_factor;

        // add moves in cm to history table
        mouse_hist_x_cm[mouse_hist_ptr] = mouse_x_cm;
        mouse_hist_y_cm[mouse_hist_ptr] = mouse_y_cm;
        mouse_hist_ptr++;
        mouse_hist_ptr %= MOUSE_NUM_AVERAGING;

        // calculate filtered moves
        mouse_filtered_x_cm = 0.0;
        mouse_filtered_y_cm = 0.0;
        for( i=0; i<MOUSE_NUM_AVERAGING; i++ ) {
            mouse_filtered_x_cm += mouse_hist_x_cm[mouse_hist_ptr];
            mouse_filtered_y_cm += mouse_hist_y_cm[mouse_hist_ptr];
        }
        mouse_filtered_x_cm /= (float)MOUSE_NUM_AVERAGING;
        mouse_filtered_y_cm /= (float)MOUSE_NUM_AVERAGING;
    //}else{
        // zero out horizontal movements
    //    mouse_x_cm = 0;
    //    mouse_y_cm = 0;
    //}

    // capture roll and pitch for next iteration
    mouse_prev_roll = roll;
    mouse_prev_pitch = pitch;

#if LOG_MOUSE
    Log_Write_Mouse(mouse_x_raw, mouse_y_raw, mouse_surface_quality, exp_change_x, exp_change_y, mouse_filtered_x_cm, mouse_filtered_y_cm);
#endif

    /*SerPri("(");
    SerPri(mouse_x_raw);
    SerPri(",");
    SerPri(mouse_y_raw);
    SerPri(",");
    SerPri(mouse_surface_quality);
    SerPri(") ex:");
    SerPri(exp_change_x,0);
    SerPri(" ey:");
    SerPri(exp_change_y,0);
    SerPri(" cx:");
    SerPri(mouse_filtered_x_cm,0);
    SerPri(" cy:");
    SerPri(mouse_filtered_y_cm,0);
    SerPri(" r:");
    SerPri(ToDeg(roll));
    SerPri(" p:");
    SerPri(ToDeg(pitch));
    SerPri(" s:");
    SerPri(press_sonar_altitude);
    SerPri(" r2p:");
    SerPri(radians_to_pixels,5);
    SerPrln();*/
}

//
// mouse_positions_control - PID loop to stop movement using mouse sensor
//       Mouse readings use right handed coordinate system:
//               +ve x = moving right, should counteract with left roll (-ve roll)
//               -ve x = moving left,  should counteract with right roll (+ve roll)
//               +ve y = moving forward,   should counteract with pitch back (+ve pitch)
//               -ve y = moving backwards, should counteract with pitch forward (-ve pitch)
//
void mouse_position_control()
{
    float mouse_err_roll, mouse_err_pitch;
    float mouse_roll_D, mouse_pitch_D;

    // ROLL
    mouse_err_roll = -mouse_filtered_x_cm; //-mouse_x_cm;  // +ve mouse reading = moving right so need to roll left (-ve roll)

    mouse_roll_I += mouse_err_roll * MOUSE_DT * KI_MOUSE_ROLL;
    mouse_roll_I = constrain(mouse_roll_I,-10,10);  // don't let I term contribute more than 10 degrees to the total

    mouse_roll_D = (mouse_err_roll - mouse_prev_err_roll) * MOUSE_DT * KD_MOUSE_ROLL;

    command_mouse_roll = KP_MOUSE_ROLL * mouse_err_roll + mouse_roll_I + mouse_roll_D;
    command_mouse_roll = constrain(command_mouse_roll, -MOUSE_MAX_ANGLE, MOUSE_MAX_ANGLE); // limit max command

    mouse_prev_err_roll = mouse_err_roll;  // store error for next iteration

    // PITCH
    mouse_err_pitch = mouse_filtered_y_cm; //mouse_y_cm;  // +ve mouse reading = moving forward so need to pitch back(+ve pitch)

    mouse_pitch_I += mouse_err_pitch * MOUSE_DT * KI_MOUSE_PITCH;
    mouse_pitch_I = constrain(mouse_pitch_I,-10,10);   // don't let I term contribute more than 10 degrees to the total

    mouse_pitch_D = (mouse_err_pitch - mouse_prev_err_pitch) * MOUSE_DT * KD_MOUSE_PITCH;

    command_mouse_pitch = KP_MOUSE_PITCH * mouse_err_pitch + mouse_pitch_I + mouse_pitch_D;
    command_mouse_pitch = constrain(command_mouse_pitch, -MOUSE_MAX_ANGLE, MOUSE_MAX_ANGLE); // limit max command

    mouse_prev_err_pitch = mouse_err_pitch;  // store error for next iteration

    // write out PIDs to log for analysis
    Log_Write_PID(11,KP_MOUSE_ROLL*mouse_err_roll,mouse_roll_I,mouse_roll_D,command_mouse_roll);
    Log_Write_PID(12,KP_MOUSE_PITCH*mouse_err_pitch,mouse_pitch_I,mouse_pitch_D,command_mouse_pitch);
}

//
// mouse_reset_I_terms_navigation - resets I terms.  should be called when navigation is switched on
//
void mouse_reset_I_terms_navigation()
{
    mouse_roll_I = 0;
    mouse_pitch_I = 0;
    mouse_prev_err_roll = 0;
    mouse_prev_err_pitch = 0;
}

