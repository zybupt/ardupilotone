/*
 * AP_BatteryVoltage.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: vgokhale
 */

#include "AP_BatteryVoltage.h"
#include "AP_ADC.h"

AP_BatteryVoltage::AP_BatteryVoltage() :
	_batteryVoltage(LOW_VOLTAGE * 1.05)
{
}

float AP_BatteryVoltage::getVoltage(int cell)
{
	switch (cell) {
	case 0:
		_batteryVoltage = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1
				+ _batteryVoltage * .9;
		break;
	case 1:
		_batteryVoltage = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1
				+ _batteryVoltage * .9;
		break;
	case 2:
		_batteryVoltage = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1
				+ _batteryVoltage * .9;
		break;
	case 3:
		_batteryVoltage = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1
				+ _batteryVoltage * .9;
		break;
	default:
		break;
		return _batteryVoltage;
	}
}

