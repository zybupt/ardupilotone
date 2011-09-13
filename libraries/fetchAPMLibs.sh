#!/bin/bash
git clone --depth 0 https://code.google.com/p/ardupilot-mega -b master
cp -r ardupilot-mega/libraries/AP_ADC .
cp -r ardupilot-mega/libraries/AP_Common .
cp -r ardupilot-mega/libraries/AP_Compass .
cp -r ardupilot-mega/libraries/AP_DCM .
cp -r ardupilot-mega/libraries/AP_GPS .
cp -r ardupilot-mega/libraries/AP_IMU .
cp -r ardupilot-mega/libraries/APM_BMP085 .
cp -r ardupilot-mega/libraries/AP_Math .
cp -r ardupilot-mega/libraries/AP_Navigation .
cp -r ardupilot-mega/libraries/AP_OpticalFlow .
cp -r ardupilot-mega/libraries/AP_RangeFinder .
cp -r ardupilot-mega/libraries/GCS_MAVLink .
cp -r ardupilot-mega/libraries/APM_PerfMon .
cp -r ardupilot-mega/libraries/DataFlash .
cp -r ardupilot-mega/libraries/FastSerial .
cp -r ardupilot-mega/libraries/APM_RC .
cp -r ardupilot-mega/libraries/GPS_MTK .
cp -r ardupilot-mega/libraries/GPS_NMEA .
cp -r ardupilot-mega/libraries/GPS_UBLOX .
rm -rf ardupilot-mega
