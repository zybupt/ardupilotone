/*
 * yourBoat.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef YOURBOAT_H_
#define YOURBOAT_H_

// vehicle options
static const apo::vehicle_t vehicle = apo::VEHICLE_BOAT;
static const apo::halMode_t halMode = apo::MODE_LIVE;
static const apo::board_t board = apo::BOARD_ARDUPILOTMEGA_2560;
static const uint8_t heartBeatTimeout = 3;
#define CONTROLLER_CLASS BoatController
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
const float steeringP = 1.0;
const float steeringI = 0.0;
const float steeringD = 0.0;
const float steeringIMax = 0.0;
const float steeringYMax = 3.0;

const float throttleP = 0.0;
const float throttleI = 0.0;
const float throttleD = 0.0;
const float throttleIMax = 0.0;
const float throttleYMax = 0.0;
const float throttleDFCut = 3.0;

#endif /* YOURBOAT_H_ */
