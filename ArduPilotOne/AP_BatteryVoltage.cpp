/*
 * AP_BatteryVoltage.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: vgokhale
 */

#include "AP_BatteryVoltage.h"
#include "AP_ADC.h"

float AP_BatteryVoltage::getVoltage()
{
	_batteryVoltage = BATTERY_VOLTAGE(analogRead(0)) * .1
			+ _batteryVoltage * .9;

	for(uint8_t i = 1; i < 4; i++)
	{
		if(_batteryVoltage > (BATTERY_VOLTAGE(analogRead(i)) * .1
				+ _batteryVoltage * .9))
		{
			_batteryVoltage = BATTERY_VOLTAGE(analogRead(i)) * .1
					+ _batteryVoltage * .9;
		}
	}

	return _batteryVoltage;
}

uint8_t AP_BatteryVoltage::batteryLeft()
{
	return 0; //TODO Define function here - calculate % left.
}

