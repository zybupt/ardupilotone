// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file	AP_RcChannelSimple.h
/// @brief	AP_RcChannelSimple manager

#ifndef AP_RcChannelSimple_h
#define AP_RcChannelSimple_h

#include <stdint.h>
#include <AP_Common.h>
#include <AP_Var.h>
#include "APM_RC.h"

namespace apo {

enum rcMode_t {
	RC_MODE_IN,
	RC_MODE_OUT,
	RC_MODE_INOUT
};

/// @class	AP_RcChannelSimple
/// @brief	Object managing one RC channel
class AP_RcChannelSimple : public AP_Var_group {
 
public:	


	/// Constructor
	AP_RcChannelSimple(AP_Var::Key key, const prog_char_t * name, APM_RC_Class & rc, const uint8_t & ch,
			const uint16_t & pwmMin,const uint16_t & pwmNeutral, const uint16_t & pwmMax,
			const rcMode_t & rcMode=RC_MODE_INOUT, const bool & reverse=false);

	// configuration
	AP_Uint8 _ch;
	AP_Uint16 _pwmMin;
	AP_Uint16 _pwmNeutral;
	AP_Uint16 _pwmMax;
	rcMode_t _rcMode;
	AP_Bool _reverse;


	// set
	uint16_t readRadio();
	void setPwm(uint16_t pwm);
	void setPosition(float position);

	// get
	uint16_t getPwm() { return _pwm; }
	float getPosition() { return _pwmToPosition(_pwm); }

	// did our read come in 50µs below the min?
	bool failSafe() { _pwm < (_pwmMin - 50); }

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

} // apo

#endif	
