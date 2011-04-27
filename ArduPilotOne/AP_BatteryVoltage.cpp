/*
 * AP_BatteryVoltage.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: vgokhale
 */

#include "AP_BatteryVoltage.h"
#include "AP_ADC.h"

void AP_BatteryVoltage::ReadBattery()
{
	battery_voltage1 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN1)) * .1 + battery_voltage1 * .9;
	battery_voltage2 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN2)) * .1 + battery_voltage2 * .9;
	battery_voltage3 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN3)) * .1 + battery_voltage3 * .9;
	battery_voltage4 = BATTERY_VOLTAGE(analogRead(BATTERY_PIN4)) * .1 + battery_voltage4 * .9;
}
