/*
 * AP_BatteryVoltage.h
 *
 *  Created on: Apr 21, 2011
 *      Author: vgokhale
 */
#include "../libraries/AP_ADC/AP_ADC.h"
#include <stdlib.h>
#include <inttypes.h>
#include <WProgram.h>
#ifndef AP_BATTERYVOLTAGE_H_
#define AP_BATTERYVOLTAGE_H_

#ifndef LOW_VOLTAGE
# define LOW_VOLTAGE			14.8
#endif

#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*VOLT_DIV_RATIO
#define VOLT_DIV_RATIO 1.0	//  Voltage divider ratio set with thru-hole resistor (see manual)
#define INPUT_VOLTAGE 5.2	// (Volts) voltage your power regulator is feeding your ArduPilot to have an accurate pressure and battery
//float 	battery_voltage 	= LOW_VOLTAGE * 1.05;		// Battery Voltage, initialized above threshold for filter

class AP_BatteryVoltage {

public:

	float _batteryVoltage;
	AP_BatteryVoltage();
	float ReadBattery();

};



#endif /* AP_BATTERYVOLTAGE_H_ */
