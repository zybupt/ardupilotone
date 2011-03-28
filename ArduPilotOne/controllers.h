#ifndef defaultControllers_H
#define defaultControllers_H

#include "ArduPilotOne.h"
#include "AP_Controller.h"
#include "AP_Var.h"
#include <avr/pgmspace.h>

#if VEHICLE_TYPE == CAR 

class CarController : public AP_Controller
{
private:
    // state
    const float * velocity;
    const float * heading;

    // control variables
    const float * headingCommand;
    const float * velocityCommand;

    // channels
    AP_RcChannel * modeCh;
    AP_RcChannel * steeringCh;
    AP_RcChannel * throttleCh;

    // control mode
    AP_Var_group _group;
    AP_Uint8 _mode;

public:
    CarController(AP_Var::Key cntrlKey, AP_Var::Key pidStrKey, AP_Var::Key pidThrKey,
            const float * heading, const float * velocity,
            const float * headingCommand, const float * velocityCommand,
            AP_RcChannel * modeCh, AP_RcChannel * steeringCh, AP_RcChannel * throttleCh) :
        heading(heading), velocity(velocity),
        headingCommand(headingCommand), velocityCommand(velocityCommand), 
        modeCh(modeCh), steeringCh(steeringCh), throttleCh(throttleCh),
        _group(cntrlKey,PSTR("CNTRL")),
        _mode(&_group,1,0,PSTR("MODE"))
    {
        Serial.println("initializing car controller");

        // steering control loop
        addBlock(new SumGain(headingCommand,&one,heading,&negativeOne));
        addBlock(new Pid(pidStrKey,PSTR("STR"),1,1,1,1,20));
        addBlock(new ToServo(steeringCh));

        // throttle control loop
        addBlock(new SumGain(velocityCommand,&one,velocity,&negativeOne));
        addBlock(new Pid(pidThrKey,PSTR("THR"),1,1,1,1,20));
        addBlock(new ToServo(throttleCh));
    }
    virtual void update(const double dt)
    {
        if (_mode == 0)

        {
            // manual control 
            steeringCh->readRadio();
            throttleCh->readRadio();
            //Serial.println("manual");
        }
        else
        {
            AP_Controller::update(dt);
            //Serial.println("automode");
        }

        //Serial.printf("steering pwm :\t");
        //Serial.printf("%7d\t",steeringCh->getPwm());
        //Serial.println();
        
        //Serial.printf("throttle pwm :\t");
        //Serial.printf("%7d\t",throttleCh->getPwm());
        //Serial.println();

        //delay(100);
    }

};

void controllerInit()
{
	_rc.push_back(new AP_RcChannel(k_chMode,PSTR("STR"),APM_RC,7,45));
	_rc.push_back(new AP_RcChannel(k_chStr,PSTR("STR"),APM_RC,0,45));
    _rc.push_back(new AP_RcChannel(k_chThr,PSTR("THR"),APM_RC,1,100));

    _controller = new CarController(k_cntrl,k_pidStr,k_pidThr,&(navigator()->yaw),
            &(navigator()->groundSpeed),
            guide()->headingCommand(), guide()->groundSpeedCommand(),_rc[0],_rc[1],_rc[2]);

}


#elif VEHICLE_TYPE == QUAD

class QuadController : public AP_Controller
{
private:
    // state
    AP_Float roll;
    AP_Float airspeed;
    AP_Float velocity;
    AP_Float heading;

    // servo positions
    AP_Float steering;
    AP_Float throttle;

    // control variables
    AP_Float headingCommand;
    AP_Float airspeedCommand;
    AP_Float rollCommand;

    // channels
    uint8_t ch1;
    uint8_t ch2;
    uint8_t ch3;
    uint8_t ch4;

public:
    QuadController(AP_Var::Key chRollKey, AP_Var::Key chPitchKey, AP_Var::Key chYawKey,
            AP_Var::Key pidRollKey, AP_Var::Key pidPitchKey, AP_Var::Key pidYawKey) :
        ch1(1), ch2(2), ch3(3), ch4(4)
    {
        // rc channels
        addCh(new AP_RcChannel(chRollKey,PSTR("ROLL"),APM_RC,chRoll,45));
        addCh(new AP_RcChannel(chPitchKey,PSTR("PTCH"),APM_RC,chPitch,45));
        addCh(new AP_RcChannel(chYawKey,PSTR("YAW"),APM_RC,chYaw,45));

        // pitch control loop
        #if AIRSPEED_SENSOR == ENABLED
            // pitch control loop w/ airspeed
            addBlock(new SumGain(airspeedCommand,AP_Float_unity,airspeed,AP_Float_negative_unity));
        #else
            // cross feed variables
            addBlock(new SumGain(roll,kffPitchCompk,throttleServo,kffT2P));
        #endif
        addBlock(new Pid(pidPitchKey,PSTR("PTCH"),0.1,0,0,1,20));
        addBlock(new Sink(chPitch));
       
        // roll control loop
        addBlock(new SumGain(headingCommand,one,heading,negOne));
        addBlock(new Pid(headingkP,headingKI,headingKD));
        addBlock(new Sink(rollCommand));
        addBlock(new SumGain(rollCommand,one,roll,negOne));
        addBlock(new Pid(rollKP,rollKI,rollKD));
        addBlock(new Sink(chRoll));

        // throttle control loop
        addBlock(new SumGain(airspeedCommand,one,airspeed,negOne));
        addBlock(new Pid(throttleKP,throttleKI,throttleKD));
        addBlock(new Sink(chThr));

        // mixing
        addBlock(new QuadMix(getRc(chRoll,chPitch,chYaw,chThr)));

        // TODO: This is just the plane controller, need to do some motor mixing here
    }
};

void controllerInit()
{
    controller = new QuadController;
}


#elif VEHICLE_TYPE == PLANE

class PlaneController : public AP_Controller
{
private:
    // state
    AP_Float roll;
    AP_Float airspeed;
    AP_Float velocity;
    AP_Float heading;

    // servo positions
    AP_Float steering;
    AP_Float throttle;

    // control variables
    AP_Float headingCommand;
    AP_Float airspeedCommand;
    AP_Float rollCommand;

    // channels
    static const uint8_t chRoll = 0;
    static const uint8_t chPitch = 1;
    static const uint8_t chYaw = 2;

public:
    PlaneController(AP_Var::Key chRollKey, AP_Var::Key chPitchKey, AP_Var::Key chYawKey,
            AP_Var::Key pidRollKey, AP_Var::Key pidPitchKey, AP_Var::Key pidYawKey) :
    {
        // rc channels
        addCh(new AP_RcChannel(chRollKey,PSTR("ROLL"),APM_RC,chRoll,45));
        addCh(new AP_RcChannel(chPitchKey,PSTR("PTCH"),APM_RC,chPitch,45));
        addCh(new AP_RcChannel(chYawKey,PSTR("YAW"),APM_RC,chYaw,45));

        // pitch control loop
        #if AIRSPEED_SENSOR == ENABLED
            // pitch control loop w/ airspeed
            addBlock(new SumGain(airspeedCommand,AP_Float_unity,airspeed,AP_Float_negative_unity));
        #else
            // cross feed variables
            addBlock(new SumGain(roll,kffPitchCompk,throttleServo,kffT2P));
        #endif
        addBlock(new Pid(pidPitchKey,PSTR("PTCH"),0.1,0,0,1,20));
        addBlock(new ToServo(getRc(chPitch)));
       
        // roll control loop
        addBlock(new SumGain(headingCommand,one,heading,negOne));
        addBlock(new Pid(headingkP,headingKI,headingKD));
        addBlock(new Sink(rollCommand));
        addBlock(new SumGain(rollCommand,one,roll,negOne));
        addBlock(new Pid(rollKP,rollKI,rollKD));
        addBlock(new ToServo(getRc(chRoll)));

        // throttle control loop
        addBlock(new SumGain(airspeedCommand,one,airspeed,negOne));
        addBlock(new Pid(throttleKP,throttleKI,throttleKD));
        addBlock(new ToServo(getRc(chThr)));
    }
};

void controllerInit()
{
    controller = new planeController;
}


#endif // VEHICLE_TYPE

#endif // defaultControllers_H

// vim:ts=4:sw=4:expandtab
