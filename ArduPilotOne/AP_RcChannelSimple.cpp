/*
	AP_RcChannelSimple.cpp - Radio library for Arduino
	Code by Jason Short, James Goppert. DIYDrones.com
	
	This library is free software; you can redistribute it and / or
		modify it under the terms of the GNU Lesser General Public
		License as published by the Free Software Foundation; either
		version 2.1 of the License, or (at your option) any later version.

*/

#include <math.h>
#include <avr/eeprom.h>
#include "AP_RcChannelSimple.h"
#include <AP_Common.h>

AP_RcChannelSimple::AP_RcChannelSimple(AP_Var::Key key, const prog_char_t * name, APM_RC_Class & rc, const uint8_t & ch,
			const uint16_t & pwmMin, 
			const uint16_t & pwmNeutral, const uint16_t & pwmMax,
			const uint16_t & pwmDeadZone,
			const bool & filter, const bool & reverse) :
		AP_Var_group(key,name),
		_rc(rc),
		ch(this,0,ch,PSTR("CH")),
		pwmMin(this,3,pwmMin,PSTR("PMIN")),
		pwmMax(this,4,pwmMax,PSTR("PMAX")),
		pwmNeutral(this,5,pwmNeutral,PSTR("PNTRL")),
		pwmDeadZone(this,6,pwmDeadZone,PSTR("PDEAD")),
		filter(this,7,filter,PSTR("FLTR")),
		reverse(this,8,reverse,PSTR("REV")),
		_pwm(0)
	{
		setPosition(0.0);
	}


uint16_t AP_RcChannelSimple::readRadio() {
	return _rc.InputCh(ch);
}

void
AP_RcChannelSimple::setPwm(uint16_t pwm)
{
	//Serial.printf("pwm in setPwm: %d\n", pwm);
	//Serial.printf("reverse: %s\n", (reverse)?"true":"false");
	
	// apply reverse
	if(reverse) pwm = int16_t(pwmNeutral-pwm) + pwmNeutral;

	//Serial.printf("pwm after reverse: %d\n", pwm);

	// apply filter
	if(filter){
		if(_pwm == 0)
			_pwm = pwm;
		else
			_pwm = ((pwm + _pwm) >> 1);		// Small filtering
	}else{
		_pwm = pwm;
	}

	//Serial.printf("pwm after filter: %d\n", _pwm);

	// apply deadzone
	_pwm = (abs(_pwm - pwmNeutral) < pwmDeadZone) ? uint16_t(pwmNeutral) : _pwm;

	// apply saturation
	if (_pwm > pwmMax) _pwm = pwmMax;
	if (_pwm < pwmMin) _pwm = pwmMin;

	//Serial.printf("pwm after deadzone: %d\n", _pwm);
	_rc.OutputCh(ch,_pwm);
}

void
AP_RcChannelSimple::setPosition(float position)
{
	if (position > 1.0) position = 1.0;
	else if (position < -1.0) position = -1.0;
	setPwm(_positionToPwm(position));
}

uint16_t
AP_RcChannelSimple::_positionToPwm(const float & position)
{
	uint16_t pwm;
	//Serial.printf("position: %f\n", position);
	float p = position - 0.0;
	if(p < 0)
		pwm = p * int16_t(pwmNeutral - pwmMin) + pwmNeutral;
	else
		pwm = p * int16_t(pwmMax - pwmNeutral) + pwmNeutral;
	constrain(pwm,uint16_t(pwmMin),uint16_t(pwmMax));
	return pwm;
}

float
AP_RcChannelSimple::_pwmToPosition(const uint16_t & pwm)
{
	float position;
	if(pwm < pwmNeutral)
		position = 1.0 * int16_t(pwm - pwmNeutral)/
			int16_t(pwmNeutral - pwmMin);
	else
		position = 1.0 * int16_t(pwm - pwmNeutral)/
			int16_t(pwmMax - pwmNeutral) ;
	constrain(position,-1.0,1.0);
	return position;
}

// ------------------------------------------
