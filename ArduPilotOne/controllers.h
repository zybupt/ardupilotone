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
        _group(cntrlKey,PSTR("CNTRL_")),
        _mode(&_group,1,0,PSTR("MODE")) {
        Serial.println("initializing car controller");

        // steering control loop
        addBlock(new SumGain(headingCommand,&one,heading,&negativeOne));
        addBlock(new Pid(pidStrKey,PSTR("STR_"),1,1,1,1,20));
        addBlock(new ToServo(steeringCh));

        // throttle control loop
        addBlock(new SumGain(velocityCommand,&one,velocity,&negativeOne));
        addBlock(new Pid(pidThrKey,PSTR("THR_"),1,1,1,1,20));
        addBlock(new ToServo(throttleCh));
    }
    virtual void update(const double dt) {
        // read mode switch
        modeCh->readRadio();
    
        // manual
        if (_mode == 0)
        {
            steeringCh->readRadio();
            throttleCh->readRadio();
            //Serial.println("manual");
        
        } else { // auto
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
    _rc.push_back(new AP_RcChannel(k_chMode,PSTR("MODE_"),APM_RC,7,45));
    _rc.push_back(new AP_RcChannel(k_chStr,PSTR("STR_"),APM_RC,0,45));
    _rc.push_back(new AP_RcChannel(k_chThr,PSTR("THR_"),APM_RC,1,100));

    _controller = new CarController(k_cntrl,k_pidStr,k_pidThr,&(navigator()->yaw),
                                    &(navigator()->groundSpeed),
                                    guide()->headingCommand(), guide()->groundSpeedCommand(),_rc[0],_rc[1],_rc[2]);

}

#elif VEHICLE_TYPE == QUAD

asdflkasdjflaskdf

/// Transform-to-Body block
class Transform : public AP_Controller::Block {
public:
    Transform(AP_Var::Key key, const prog_char_t * name,
    float * commandRoll,
    float * commandPitch,
    float * commandYaw,
             ) :
        _group(key,name),
        _cR(&_group,1,cR,PSTR("roll" )),
        _cP(&_group,2,cP,PSTR("pitch")),
        _cY(&_group,3,cY,PSTR("yaw"  ))
    {
        _output.push_back(new float(0.0));
        _output.push_back(new float(0.0));
    }
    virtual void update(const float & dt) {
        float trigSin = sin (_cY*(-1));
        float trigCos = cos (_cY*(-1));
        output(0) = _cR * _trigCos + _cP * _trigSin;
        output(1) = _cR * _trigSin + _cP * _trigCos;
    }
private:
    AP_Float cR;
    AP_Float cP;
    AP_Float cY;
};

/// PID block
class PidDFB : public AP_Controller::Block {
public:
    Pid(AP_Var::Key key, const prog_char_t * name,
    float kP = 0.0,
    float kI = 0.0,
    float kD = 0.0,
    float iMax = 0.0,
    float * derivative
       ) :
        _group(key,name),
        _eP(0),
        _eI(0),
        _eD(0),
addBlock(new TrigFunctSin(commandYaw*(-1));
         addBlock(new TrigFunctCos(commandYaw*(-1));
                  addBlock(new SumGain(eastTilt*TrigFunctCos,one,northTilt*TrigFunctSin,negone));
                  addBlock(new SumGain(eastTilt*TrigFunctSin,one,northTilt*TrigFunctCos,   one));


        _kP(&_group,1,kP,PSTR("P")),
        _kI(&_group,2,kI,PSTR("I")),
        _kD(&_group,3,kD,PSTR("D")),
        _iMax(&_group,4,iMax,PSTR("IMAX")),
        _derivative(derivative) {
        // create output
        _output.push_back(new float(0.0));
    }

    virtual void update(const float & dt) {
        if (_output.getSize() < 1 || (!_input[0]) || (!_output[0]) ) return;

        // proportional, note must come after derivative
        // because derivatve uses _eP as previous value
        _eP = input(0);

        // integral
        _eI += _eP*dt;

        // wind up guard
        if (_eI > _iMax) _eI = PID_POS_AWU; //_iMax;                            // <--------changed to PID anti-windup
        else if (_eI < -_iMax) _eI = -PID_POS_AWU; //-_iMax;                        // <--------changed to PID anti-windup

        // pid sum
        output(0) = _kP*_eP + _kI*_eI - _kD*_d;

        // debug output
        /*
        Serial.println("kP, kI, kD: ");
        Serial.print(_kP,5); Serial.print(" ");
        Serial.print(_kI,5); Serial.print(" ");
        Serial.println(_kD,5);
        Serial.print("eP, eI, eD: ");
        Serial.print(_eP,5); Serial.print(" ");
        Serial.print(_eI,5); Serial.print(" ");
        Serial.println(_eD,5);
        Serial.print("input: ");
        Serial.println(input(0),5);
        Serial.print("output: ");
        Serial.println(output(0),5);
        */
    }
private:
    AP_Var_group _group; /// helps with parameter management
    AP_Float _eP; /// input
    AP_Float _eI; /// integral of input
    AP_Float _eD; /// derivative of input
    AP_Float _kP; /// proportional gain
    AP_Float _kI; /// integral gain
    AP_Float _kD; /// derivative gain
    AP_Float _iMax; /// integrator saturation
    float * _derivative; // derivative fed back
};

class QuadController: public AP_Controller {
private:

    // state
    AP_FLoat airspeed;
    AP_Float velocity;
    AP_Float yaw;
    AP_Float pitch;
    AP_Float roll;

    // servo positions
    AP_Float steering;
    AP_Float throttle;

    // control variables
    AP_Float yawCommand;
    AP_Float airspeedCommand;
    AP_Float rollCommand;

    // position variables (UPDATED
    AP_Float commandNorth;
    AP_Float commandEast;
    AP_Float commandDown;
    AP_Float OutputNorth;
    AP_Float OutputEast;
    AP_Float OutputDown;
    AP_Float DEV_OutputNorth;
    AP_Float DEV_OutputEast;
    AP_Float DEV_OutputDown;
    AP_Float PID_POS_LIM;

    // transform to body att.

    // attitude variables (UPDATED)
    AP_Float pitchMotorMix;
    AP_Float rollMotorMix;

    // mix manual variables (UPDATED)
    AP_Float MIX_REMOTE_WEIGHT;
    AP_Float MIX_OFFSET_WEIGHT;
    AP_Float ATT_OFFSET_X;
    AP_Float ATT_OFFSET_Y;
    AP_Float ATT_OFFSET_Z;

    // QUAD controller output
    AP_Float THRUST_HOVER_OFFSET;

    // mix variables (UPDATED)
    AP_Float thrustMix;
    AP_Float rollMix;
    AP_Float yawMix;
    AP_Float pitchMix;

    // satruation (UPDATED)
    AP_Float motorMin;
    AP_Float motorMax;

    // channels (UPDATED)
    unint8_t chLeft;
    unint8_t chRight;
    unint8_t chFront;
    unint8_t chBack;

public:

    QuadController( AP_Var::Key chRollKey, AP_Var::Key chPitchKey, AP_Var::Key chYawKey,
    AP_Var::Key pidRollKey,AP_Var::Key pidPitchkey,AP_Var::Key pidYawKey):
        ch1(1),ch2(2),ch3(3),ch4(4) {

        // RC channels
        addCh(new AP_RcChannel(chRollKey, PSTR("ROLL"), APM_RC, chRoll, 45));
        addCh(new AP_RcChannel(chPitchKey,PSTR("PITCH"),APM_RC, chPitch,45);
        addCh(new AP_RcChannel(chYawKey,  PSTR("YAW"),  APM_RC, chYaw,  45));

        // pitch control loop
#if AIRSPEED_SENSOR == ENABLED
        // pitch control loop with airspeed
        addBlock(new SumGain(airspeedCommand, AP_Float_unity, airspeed, AP_Float_negative_unity));
#else
        // cross feed variables
        addBlock(new SumGain(roll, kffPitchCompk, throttleServo, kffT2P));
#endif
        //  addBlock(new Pid (pidPitchKey,PSTR("PITCH"),0.1,0,0,1,20));
        //  addBlock(new Sink(chPitch));

        //  // roll control loop
        //  addBlocK(new SumGain(headingCommand, one, heading, negOne));
        //  addBlock(new Pid(headingKP, headingKI, headingKD));
        //  addBlock(new Sink(rollCommand));
        //  addBlock(new SumGain(rollCommand, one, roll, negone));
        //  addBlock(new Pid(rollKP, rollKI, rollKD));
        //  addBlock(new Sink(chThr));

        //  // throttle control
        //  addBlock(new SumGain(airspeedCommand, one, airspeed, negOne));
        //  addBlock(new Pid(throttleKP, throttleKI, throttleKD));
        //  addBlock(new Sink(chThr));

//------>clean up position control

        // position control (UPDATED)
        addBlock(new SumGain(ReferenceYaw,one,northTilt,negone,eastTilt,one,thurstMix,negone));     //DOUBLE CHECK THE INPUTS TO THIS LINE
        //  +- NORTH
        addBlock(new Pid(commandNorth, OutputNorth, DEV_OutputNorth, PID_POS_LIM));
        addBlock(new Saturate(-PID_POS_LIM,PID_POS_LIM));
        //  +- EAST
        addBlock(new Pid(commandEast, OutputEast, DEV_OutputEast, PID_POS_LIM));
        addBlock(new Saturate(-PID_POS_LIM,PID_POS_LIM));
        //  +- DOWN
        addBlock(new Pid(commandDown, OutputDown, DEV_OutputDown, PID_POS_LIM));
        addBlock(new Saturate(-PID_POS_LIM,PID_POS_LIM));

//----->edit from here down
        // transform to body att. (UPDATED)
        addBlock(new Transform(commandRoll,commandPitch,commandYaw));

        // thrust trim adjust (UPDATED)
        addBlock(new SumGain(ReferencePitch,one,ReferenceRoll,one,ReferenceYaw,one));

        // mix manual (UPDATED)
        addBlock(new SumGain(commandPitch*MIX_REMOTE_WEIGHT,one,
        commandRoll *MIX_REMOTE_WEIGHT,one,
        commandYaw  *MIX_REMOTE_WEIGHT,one,
        ATT_OFFSET_X*MIX_OFFSET_WEIGHT,negone,
        ATT_OFFSET_Y*MIX_OFFSET_WEIGHT,negone,
        ATT_OFFSET_Z*MIX_OFFSET_WEIGHT,negone));
//----->
// create Blocks with no input, no output; they simply pass the adjusted variables

// input: commandNorth,commandEast,commandDown,
// output: pitch, roll, yaw, thrust

//----->edit from here up

        // attitude control (UPDATED)
        addBlock(new SumGain(commandPitch,one,outputPitch,negOne);
        addBlock(new PidDFB(PID_ATT_P,PID_ATT_I,PID_ATT_D));
        addBlock(new sink(rollMix));
        addBlock(new PidDFB(PID_ATT_P,PID_ATT_I,PID_ATT,D));
        addBlock(new sink(pitchMix));
        addBlock(new Pid(PID_YAWPOS_P,PID_YAWPOS_I,PID_YAWPOS_D));
        addBlock(new Pid(PID_YAWSPEED_P,PID_YAWSPEED_I,PID_YAWSPEED,D));
        addBlock(new sink(yawMix));

        // thrust trim (UPDATED)
        addBlock(new SumGain(THRUST_HOVER_OFFSET,one,thrustMix,one));

        // motor mix (UPDATED)
        addBlock(new QuadMix(getRc(chRoll, chPitch, chYaw, chThr)));
        //  Left Motor
        addBlock(new SumGain(thrustMix,one,rollMix,one,yawMix,one));
        addBlock(new Saturate(motorMin,motorMax);
        addBlock(new ToServo(chLeft);
        //  Right Motor
        addBlock(new SumGain(thrustMix,one,rollMix,negone,yawMix,one));
        addBlock(new Saturate(motorMin,motorMax);
        addBlock(new ToServo(chRight);
        //  Front Motor
        addBlock(new SumGain(thrustMix,one,pitchMix,one,yawMix,negone));
        addBlock(new Saturate(motorMin,motorMax);
        addBlock(new ToServo(chFront);
        //  Back Motor
        addBlock(new SumGain(thrustMix,one,pitchMix,negone,yawMix,negone));
        addBlock(new Saturate(motorMin,motorMax);
        addBlock(new ToServo(chBack);
    }
};

void controllerInt()
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
                    AP_Var::Key pidRollKey, AP_Var::Key pidPitchKey, AP_Var::Key pidYawKey) : {
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
