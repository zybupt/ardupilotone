/*
 * easystar.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef EASYSTAR_H_
#define EASYSTAR_H_

// vehicle options
static const apo::vehicle_t vehicle = apo::VEHICLE_PLANE;
static const apo::halMode_t halMode = apo::MODE_LIVE;
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_2560;
static const uint8_t heartBeatTimeout = 3;
#define CONTROLLER_CLASS PlaneController
#define RANGE_FINDER_CLASS AP_RangeFinder_MaxsonarLV

// optional sensors
static bool gpsEnabled = false;
static bool baroEnabled = true;
static bool compassEnabled = true;

static bool rangeFinderFrontEnabled = true;
static bool rangeFinderBackEnabled = true;
static bool rangeFinderLeftEnabled = true;
static bool rangeFinderRightEnabled = true;
static bool rangeFinderUpEnabled = true;
static bool rangeFinderDownEnabled = true;

// loop rates
static const float loop0Rate = 150;
static const float loop1Rate = 100;
static const float loop2Rate = 10;
static const float loop3Rate = 1;
static const float loop4Rate = 0.1;

// gains
static const float rdrAilMix = 1.0; // since there are no ailerons

// bank error to roll servo
static const float pidBnkRllP = 0.5;
static const float pidBnkRllI = 0.0;
static const float pidBnkRllD = 0.0;
static const float pidBnkRllAwu = 0.0;
static const float pidBnkRllLim = 1.0;
static const float pidBnkRllDFCut = 0.0;

// pitch error to pitch servo
static const float pidPitPitP = 0.5;
static const float pidPitPitI = 0.0;
static const float pidPitPitD = 0.0;
static const float pidPitPitAwu = 0.0;
static const float pidPitPitLim = 1.0;
static const float pidPitPitDFCut = 0.0;

// speed error to pitch command
static const float pidSpdPitP = 0.1;
static const float pidSpdPitI = 0.0;
static const float pidSpdPitD = 0.0;
static const float pidSpdPitAwu = 0.0;
static const float pidSpdPitLim = 1.0;
static const float pidSpdPitDFCut = 0.0;

// yaw rate error to yaw servo
const float pidYwrYawP = 0.5;
const float pidYwrYawI = 0.0;
const float pidYwrYawD = 0.0;
const float pidYwrYawAwu = 0.0;
const float pidYwrYawLim = 1.0;
const float pidYwrYawDFCut = 0.0;

// heading error to bank angle command
const float pidHdgBnkP = 0.0;
const float pidHdgBnkI = 0.0;
const float pidHdgBnkD = 0.0;
const float pidHdgBnkAwu = 0.0;
const float pidHdgBnkLim = 0.0;
const float pidHdgBnkDFCut = 0.0;

// altitude error to throttle command
const float pidAltThrP = 0.1;
const float pidAltThrI = 0.0;
const float pidAltThrD = 0.0;
const float pidAltThrAwu = 0.0;
const float pidAltThrLim = 0.1;
const float pidAltThrDFCut = 0.0;

// trim control positions (-1,1)
const float ailTrim = 0.0;
const float elvTrim = 0.0;
const float rdrTrim = 0.0;
const float thrTrim = 0.2;

#endif /* EASYSTAR_H_ */
