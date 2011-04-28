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


class AP_BatteryVoltage {

public:
	int _batteryVoltage;
	int ReadBattery();
};



#endif /* AP_BATTERYVOLTAGE_H_ */
