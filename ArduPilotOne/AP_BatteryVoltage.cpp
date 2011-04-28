/*
 * AP_BatteryVoltage.cpp
 *
 *  Created on: Apr 21, 2011
 *      Author: vgokhale
 */

#include "AP_BatteryVoltage.h"
#include "AP_ADC.h"
#include <Wire.h>
#include <AP_RangeFinder.h>     // Range finder library

int AP_BatteryVoltage::ReadBattery()
{
	AP_RangeFinder_MaxsonarLV a0, a1, a2, a3;
	AP_ADC_ADS7844	adc;
	adc.Init();            // APM ADC library initialization

	a0.init(0, &adc);
	a1.init(1, &adc);
	a2.init(2, &adc);
	a3.init(3, &adc);


//	_batteryVoltage = (analogRead(0)*5/1023)/0.28;
//	for(uint8_t i = 1; i < 4; i++)
//	{
//		if(_batteryVoltage > (analogRead(i)*5/1023)/0.28)
//			_batteryVoltage = (analogRead(i)*5/1023)/0.28;
//	}

		_batteryVoltage = (analogRead(0)*5/1023/0.28);/* + _batteryVoltage * 0.9;*/
//		for(uint8_t i = 1; i < 3; i++)
//		{
//			if(_batteryVoltage > (analogRead(i)))
//				_batteryVoltage = analogRead(i);
//		}

	return _batteryVoltage;

}
