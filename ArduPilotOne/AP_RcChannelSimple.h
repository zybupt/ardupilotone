// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RcChannelSimple.h
/// @brief	AP_RcChannelSimple manager

#ifndef AP_RcChannelSimple_h
#define AP_RcChannelSimple_h

#include <stdint.h>
#include <AP_Common.h>
#include <AP_Var.h>
#include "APM_RC.h"

/// @class	AP_RcChannelSimple
/// @brief	Object managing one RC channel
class AP_RcChannelSimple : public AP_Var_group {
 
public:	

	/// Constructor
	AP_RcChannelSimple(AP_Var::Key key, const prog_char_t * name, APM_RC_Class & rc, const uint8_t & ch,
			const uint16_t & pwmMin=1200, 
			const uint16_t & pwmNeutral=1500, const uint16_t & pwmMax=1800,
			const uint16_t & pwmDeadZone=1,
			const bool & filter=true, const bool & reverse=false);

	// configuration
	AP_Uint8 ch;
	AP_Uint16 pwmMin;
	AP_Uint16 pwmNeutral;
	AP_Uint16 pwmMax;
	AP_Uint16 pwmDeadZone;
	AP_Bool filter;
	AP_Bool reverse;

	// set
	uint16_t readRadio();
	void setPwm(uint16_t pwm);
	void setPosition(float position);

	// get
	uint16_t getPwm() { return _pwm; }
	float getPosition() { return _pwmToPosition(_pwm); }

	// did our read come in 50µs below the min?
	bool failSafe() { _pwm < (pwmMin - 50); }

private:

	// configuration
	const char * _name;
	APM_RC_Class & _rc;
		
	// internal states
	uint16_t _pwm; // this is the internal state, position is just created when needed

	// private methods
	uint16_t _positionToPwm(const float & position);
	float _pwmToPosition(const uint16_t & pwm);
};

#endif	
