/*
 * AP_BatteryVoltage.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: vgokhale
 */

#include "AP_BatteryVoltage.h"
#include "AP_ADC.h"

float AP_BatteryVoltage::ReadBattery()
{
	_batteryVoltage = analogRead(0)*5/1023/0.71;
	for(uint8_t i = 1; i < 4; i++)
	{
		if(_batteryVoltage > (analogRead(i)*5/1023/0.71))
			_batteryVoltage = analogRead(i)*5/1023/0.71;
	}

	return _batteryVoltage;
}
