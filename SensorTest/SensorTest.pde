/*
  AP_RangeFinder_test
  Code by DIYDrones.com
*/

#include <AP_RangeFinder.h>     // Range finder library
#include <AP_ADC.h>		// ArduPilot Mega Analog to Digital Converter Library

//#define RF_PIN AP_RANGEFINDER_PITOT_TUBE // the pitot tube on the front of the oilpan
//#define RF_PIN A5 // A5 is the far back-right pin on the oilpan (near the CLI switch)


// declare global instances for reading pitot tube
AP_ADC_ADS7844	adc;

// create the range finder object
//AP_RangeFinder_SharpGP2Y aRF;
//AP_RangeFinder_MaxsonarXL aRF;
AP_RangeFinder_MaxsonarLV aRF0;
AP_RangeFinder_MaxsonarLV aRF1;

void setup()
{
  Serial.begin(115200);
  Serial.println("Range Finder Test v1.0");
  adc.Init();            // APM ADC library initialization
  aRF0.init(0, &adc);
  aRF1.init(1, &adc);
}

void loop()
{
    int i = 0;

    Serial.print("dist0:");
    Serial.print(aRF0.read());

    Serial.print("\traw0:");
    Serial.print(aRF0.raw_value);

    Serial.println();

    Serial.print("dist1:");
    Serial.print(aRF1.read());

    Serial.print("\traw1:");
    Serial.print(aRF1.raw_value);

    Serial.println();

    delay(20);
}

