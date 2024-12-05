#include "Motor.h"


Motor::Motor(int pwmPin, int enablePin)
    :_pwmPin(pwmPin), _enablePin(enablePin), _desiredPos(0), _previousError(0), _previousControl(0), _kp(3.0), _kd(0.5), _tau(0.001),  _maxCurrent(1), _fCutoff(25), _wc(2*PI*_fCutoff) {
    pinMode(_pwmPin,OUTPUT);
    pinMode(_enablePin,OUTPUT);
    digitalWrite(_enablePin,1);                             //Activate the motor

    }

void Motor::drive(float controlSignal){

    float dutyCycle = abs(controlSignal+_maxCurrent)/(2*_maxCurrent);
    dutyCycle = constrain(dutyCycle,0.1,0.9);
    analogWrite(_pwmPin,512*dutyCycle);

}

void Motor::stop(){

    analogWrite(_pwmPin,256);                           //PWM ranges from 0-512. 0 corresponds to highest negative current, 512 corresponds to highest positive current. 256 is 0.

}

float Motor::wheelDesired(float theta){

    float alpha, h;
    if (theta>0){
        alpha = _alpha1;
        h = _l/cos(alpha);
    }

    else if (theta <0){

        alpha = -_alpha2;
        h = _l/cos(alpha);

    }
    else{
        alpha = 0;
        h = _l;
    }

    return _d*cos(theta) + h*sin(theta-alpha);

}

float Motor::getCompression(float theta, float thetadot, float theta_r1){

    return 0;
}



void Motor::setDesiredPosition(float desired_pos){

    _desiredPos = desired_pos;                              

}


void Motor::control(float currentPos){

    float error = _desiredPos - currentPos;                                             //Calculate the error at current time step
    float controlSignal =   error*(_kp+2*_kd*_wc)/(2+_wc*_tau) - 3.165*_previousError -     (_wc-(2/_tau))/(_wc+(2/_tau))*_previousControl;         //CHANGE MIDDLE TERM!! Discrete bilinear PD controler
    controlSignal = constrain(controlSignal,-_maxCurrent,_maxCurrent);                  //Saturate control current
    drive(controlSignal);                                                               //Drive the motor
    _previousError = error;                                                             //Update previous time step error
    _previousControl = controlSignal;                                                   //Update current time step error

}