#include <Arduino.h>
#include "I2Cdev.h"
#include "Motor.h"
#include <Encoder.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "IMUManager.h"


// Encoder Pins
const int encoderPinsA[4] = {34, 39, 31, 29}; // Encoder A pins
const int encoderPinsB[4] = {35, 40, 32, 30}; // Encoder B pins


// Motor Pins
const int pwmPins[4] = {1, 3, 5, 7};        // PWM pins for motors
const int enablePins[4] = {0, 2, 4, 6};       // Enable pins for motors


// Define the 4 encoders
Encoder encoders[4] = {
    Encoder(encoderPinsA[0], encoderPinsB[0]),
    Encoder(encoderPinsA[1], encoderPinsB[1]),
    Encoder(encoderPinsA[2], encoderPinsB[2]),
    Encoder(encoderPinsA[3], encoderPinsB[3])
};

// Define the 4 Motors
Motor motors[4] = {
    Motor(pwmPins[0], enablePins[0]),
    Motor(pwmPins[1], enablePins[1]),
    Motor(pwmPins[2], enablePins[2]),
    Motor(pwmPins[3], enablePins[3])
};


// Define the 3 IMUs

MPU6050 mpu1(0x69);
MPU6050 mpu2(0x68, &Wire1);
MPU6050 mpu3(0x68, &Wire2);
IMUManager imuManager(mpu1, mpu2, mpu3);



// Define the control loop object
IntervalTimer controlTimer;

// Desired and current positions are set to 0 for initialization
float desiredPositions[4] = {0.0, 0.0, 0.0, 0.0};
float currentPositions[4] = {0.0, 0.0, 0.0, 0.0};

//Encoder pulses per revolution for the AMS AS5047P encoder
float ppr = 1000;

float f_cutoff = 25;
float wc  = 2*PI*f_cutoff;
float tau = 0.001;
//Bodu angle, leg angle, body angular velocity are defined
float theta, theta_r1, thetadot, theta_r2, theta_previous, thetadot_previous;

void control_loop(){

        thetadot = (2*theta*wc - 2*wc*theta_previous-(tau*wc-2)*thetadot_previous)/(2+tau*wc);
        thetadot_previous = thetadot;
        theta_previous = theta;

    for (int i = 0; i < 4; i++) {

        currentPositions[i] = encoders[i].read()*PI/(2*ppr);          // Read motor position

        //theta = roll;


        // Set Desired Positions
        switch(i){
            case 1:
              float wheel_despos = motors[1].wheelDesired(theta);
              motors[1].setDesiredPosition(wheel_despos);
              motors[3].setDesiredPosition(wheel_despos);
            case 2:
              float compression = motors[2].getCompression(theta, thetadot, theta_r1);
              motors[2].setDesiredPosition(compression);
              motors[4].setDesiredPosition(compression);         

        }





        motors[i].control(currentPositions[i]);                       // Drive the motor

        //Print Desired and Measured Positions
        Serial.print("Motor ");
        Serial.print(i);
        Serial.print(" | Current Position: ");
        Serial.print(currentPositions[i]);
        Serial.print(" | Desired Position: ");
        Serial.println(desiredPositions[i]);
    }




}


void setup() {

  Serial.begin(38400);

  // Wait for Serial to connect
  while (!Serial);  

  imuManager.initializeIMUs();
  
  //Initialize motors

  for (int i = 0; i < 4; i++) {

    motors[i].setDesiredPosition(0);
  
  }
  

  //Initialize control loop
  controlTimer.begin(control_loop,1000);


}

void loop() {

  imuManager.readIMUData();
  theta = imuManager.roll1;
  theta_r1 = imuManager.roll2;
  theta_r2 = imuManager.roll3;


}


