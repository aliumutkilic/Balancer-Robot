#ifndef IMUMANAGER_H
#define IMUMANAGER_H

#include "MPU6050_6Axis_MotionApps20.h"

class IMUManager {
public:
    IMUManager(MPU6050 &mpu1, MPU6050 &mpu2, MPU6050 &mpu3);
    void initializeIMUs();
    void readIMUData();
    void printRolls();
    float roll1, roll2, roll3;
private:
    MPU6050 &mpu1;
    MPU6050 &mpu2;
    MPU6050 &mpu3;
    bool dmpReady;
    uint16_t packetSize1, packetSize2, packetSize3;
    uint8_t fifoBuffer[64];
    Quaternion q1, q2, q3;
    VectorFloat gravity1, gravity2, gravity3;
    float ypr1[3], ypr2[3], ypr3[3];
    
};

#endif