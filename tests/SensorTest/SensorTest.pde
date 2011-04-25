/* herro1s
  AP_RangeFinder_test
  Code by DIYDrones.com
*/
#include <Wire.h>
#include <AP_RangeFinder.h>     // Range finder library
#include <AP_ADC.h>		// ArduPilot Mega Analog to Digital Converter Library
#include <APM_BMP085.h> // ArduPilot Mega BMP085 Library

APM_BMP085_Class APM_BMP085;


//#define RF_PIN AP_RANGEFINDER_PITOT_TUBE // the pitot tube on the front of the oilpan
//#define RF_PIN A5 // A5 is the far back-right pin on the oilpan (near the CLI switch)


// declare global instances for reading pitot tube
AP_ADC_ADS7844	adc;

// create the range finder object
//AP_RangeFinder_SharpGP2Y aRF;
//AP_RangeFinder_MaxsonarXL aRF;
AP_RangeFinder_MaxsonarLV aRF0;
AP_RangeFinder_MaxsonarLV aRF1;
unsigned long timer;

void setup()
{


	Serial.begin(115200);
	Serial.println("Range Finder Test v1.0");
	APM_BMP085.Init();	 // APM ADC initialization
	adc.Init();            // APM ADC library initialization
	aRF0.init(5, &adc);
	aRF1.init(4, &adc);
	delay(1000);
	timer = millis();
}

void loop()
{
    int i = 0;
	int ch;
	float tmp_float;
	float Altitude;

    Serial.print("dist0:");	//the APM board also has its own onboard power supply so you can power it with a separate battery
    Serial.print(aRF0.read()/2);		//8-bit sensors would require a division by 2.....Vcc/1024 (for 10 bit) and Vcc/512 (for 8 bit)

    Serial.print("\traw0:");
    Serial.print(aRF0.raw_value/2);

    Serial.println();

    Serial.print("dist1:");
    Serial.print(aRF1.read()/2);

    Serial.print("\traw1:");
    Serial.print(aRF1.raw_value/2);

    Serial.println();
	if((millis()- timer) > 50){
		timer = millis();
		APM_BMP085.Read();
	    Serial.print("Pressure:");
		Serial.print(APM_BMP085.Press);
	    Serial.print(" Temperature:");
		Serial.print(APM_BMP085.Temp / 10.0);
	    Serial.print(" Altitude:");
		tmp_float = (APM_BMP085.Press / 101325.0);
		tmp_float = pow(tmp_float, 0.190295);
		Altitude = 44330 * (1.0 - tmp_float);
		Serial.print(Altitude);
		Serial.println();
	}
    delay(200);
}

