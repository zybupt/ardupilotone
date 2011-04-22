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
# define LOW_VOLTAGE			11.4
#endif


#define BATTERY_PIN1 0		        // These are the pins for the voltage dividers
#define BATTERY_PIN2 1
#define BATTERY_PIN3 2
#define BATTERY_PIN4 3

#define VOLTAGE_PIN_0 0 // These are the pins for current sensor: voltage
#define CURRENT_PIN_1 1 // and current

#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*VOLT_DIV_RATIO
#define VOLT_DIV_RATIO 1.0	//  Voltage divider ratio set with thru-hole resistor (see manual)
#define INPUT_VOLTAGE 5.2	// (Volts) voltage your power regulator is feeding your ArduPilot to have an accurate pressure and battery
#define CURRENT_AMPS(x) (x*(INPUT_VOLTAGE/1024.0))*CURR_AMP_DIV_RATIO
float 	battery_voltage 	= LOW_VOLTAGE * 1.05;		// Battery Voltage, initialized above threshold for filter

class AP_BatteryVoltage {

public:
	int battery_voltage1, battery_voltage2, battery_voltage3, battery_voltage4,current_amps;
	AP_BatteryVoltage();
	void ReadBattery();
	virtual ~AP_BatteryVoltage();
};



#endif /* AP_BATTERYVOLTAGE_H_ */
