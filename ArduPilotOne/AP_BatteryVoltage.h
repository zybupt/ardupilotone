/*
 * AP_BatteryVoltage.h
 *
 *  Created on: Apr 21, 2011
 *      Author: vgokhale
 */
#ifndef AP_BATTERYVOLTAGE_H_
#define AP_BATTERYVOLTAGE_H_

#include "../libraries/AP_ADC/AP_ADC.h"
#include <stdlib.h>
#include <inttypes.h>
#include <WProgram.h>


#ifndef LOW_VOLTAGE
# define LOW_VOLTAGE			14
#endif


#define BATTERY_PIN1 0		        // These are the pins for the voltage dividers
#define BATTERY_PIN2 1
#define BATTERY_PIN3 2
#define BATTERY_PIN4 3

#define VOLTAGE_PIN_0 0 // These are the pins for current sensor: voltage


#define BATTERY_VOLTAGE(x) (x*(INPUT_VOLTAGE/1024.0))*VOLT_DIV_RATIO
#define VOLT_DIV_RATIO 1.0	//  Voltage divider ratio set with thru-hole resistor (see manual)
#define INPUT_VOLTAGE 5.2	// (Volts) voltage your power regulator is feeding your ArduPilot to have an accurate pressure and battery




class AP_BatteryVoltage {
public:
	float getVoltage(int cell);
	AP_BatteryVoltage();
private:
	float _batteryVoltage;
};

#endif /* AP_BATTERYVOLTAGE_H_ */
