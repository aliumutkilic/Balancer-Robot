#ifndef Motor_h
#define Motor_h


#include <Arduino.h>

class Motor {

public:

    Motor(int pwmPin, int enablePin);       //Motor is defined by pwm and enable pins
    void drive(float dutyCycle);            // Drive motor with a given duty cycle
    void stop();                            //Terminate
    float wheelDesired(float theta);
    void setDesiredPosition(float desired_pos);        //Set desired motor position through the control algo suggested
    void control(float currentPos);                                             //Joint-level PD Controller
    float getCompression(float theta, float thetadot, float theta_r1);

private:

    int _pwmPin;                            //PWM pin of the motor driver
    int _enablePin;                         //Enable pin of the motor driver
    float _desiredPos;                      //Desired position
    float _previousError;                   //Error at previous step
    float _previousControl;                 //Control at previous step
    float _kp;                              //Proportional gain
    float _kd;                              //Derivative gain
    float _tau;
    float _maxCurrent;
    float _fCutoff;
    float _wc;
    float _d;
    float _l;
    float _alpha1;
    float _alpha2;

};


#endif

