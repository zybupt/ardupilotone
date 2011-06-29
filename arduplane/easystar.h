/*
 * easystar.h
 *
 *  Created on: May 1, 2011
 *      Author: jgoppert
 */

#ifndef EASYSTAR_H_
#define EASYSTAR_H_

const float rdrAilMix = 1.0; // since there are no ailerons

// bank error to roll servo
const float pidBnkRllP = 0.5;
const float pidBnkRllI = 0.0;
const float pidBnkRllD = 0.0;
const float pidBnkRllAwu = 0.0;
const float pidBnkRllLim = 1.0;
const float pidBnkRllDFCut = 0.0;

// pitch error to pitch servo
const float pidPitPitP = 0.5;
const float pidPitPitI = 0.0;
const float pidPitPitD = 0.0;
const float pidPitPitAwu = 0.0;
const float pidPitPitLim = 1.0;
const float pidPitPitDFCut = 0.0;

// speed error to pitch command
const float pidSpdPitP = 0.1;
const float pidSpdPitI = 0.0;
const float pidSpdPitD = 0.0;
const float pidSpdPitAwu = 0.0;
const float pidSpdPitLim = 1.0;
const float pidSpdPitDFCut = 0.0;

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
