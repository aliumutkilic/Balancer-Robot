#include "IMUManager.h"
#include <Arduino.h>
#include "Wire.h"


IMUManager::IMUManager(MPU6050 &mpu1, MPU6050 &mpu2, MPU6050 &mpu3)
    : mpu1(mpu1), mpu2(mpu2), mpu3(mpu3), dmpReady(false) {}

void IMUManager::initializeIMUs() {
    Wire.begin();
    Wire1.begin();
    Wire2.begin();
    
    Wire.setClock(400000);
    Wire1.setClock(400000);
    Wire2.setClock(400000);

    Serial.println(F("Initializing IMUs..."));
    
    // Initialize IMUs
    mpu1.initialize();
    mpu2.initialize();
    mpu3.initialize();

    Serial.println(mpu1.testConnection() ? F("MPU6050-1 connection successful") : F("MPU6050-1 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050-2 connection successful") : F("MPU6050-2 connection failed"));
    Serial.println(mpu3.testConnection() ? F("MPU6050-3 connection successful") : F("MPU6050-3 connection failed"));

    // Set the gyro and accel offsets for each MPU (based on your original code)
    mpu1.setXGyroOffset(220);
    mpu1.setYGyroOffset(76);
    mpu1.setZGyroOffset(-85);
    mpu1.setZAccelOffset(1788);

    mpu2.setXGyroOffset(220);
    mpu2.setYGyroOffset(76);
    mpu2.setZGyroOffset(-85);
    mpu2.setZAccelOffset(1788);

    mpu3.setXGyroOffset(220);
    mpu3.setYGyroOffset(76);
    mpu3.setZGyroOffset(-85);
    mpu3.setZAccelOffset(1788);

    // Initialize DMP
    uint8_t devStatus1 = mpu1.dmpInitialize();
    uint8_t devStatus2 = mpu2.dmpInitialize();
    uint8_t devStatus3 = mpu3.dmpInitialize();

    if (devStatus1 == 0 && devStatus2 == 0 && devStatus3 == 0) {
        Serial.println(F("Calibrating..."));
        mpu1.CalibrateAccel(6);
        mpu1.CalibrateGyro(6);
        mpu1.PrintActiveOffsets();

        mpu2.CalibrateAccel(6);
        mpu2.CalibrateGyro(6);
        mpu2.PrintActiveOffsets();

        mpu3.CalibrateAccel(6);
        mpu3.CalibrateGyro(6);
        mpu3.PrintActiveOffsets();

        mpu1.setDMPEnabled(true);
        mpu2.setDMPEnabled(true);
        mpu3.setDMPEnabled(true);

        packetSize1 = mpu1.dmpGetFIFOPacketSize();
        packetSize2 = mpu2.dmpGetFIFOPacketSize();
        packetSize3 = mpu3.dmpGetFIFOPacketSize();

        dmpReady = true;
        Serial.println(F("IMUs are ready!"));
    } else {
        Serial.println(F("DMP initialization failed."));
    }
}

void IMUManager::readIMUData(){

    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu1.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 



    
            // display Euler angles in degrees
            mpu1.dmpGetQuaternion(&q1, fifoBuffer);
            mpu1.dmpGetGravity(&gravity1, &q1);
            mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
            roll1 = ypr1[2]*180/M_PI;
    }

        if (mpu2.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 



    
            // Obtain Euler angles in degrees
            mpu2.dmpGetQuaternion(&q2, fifoBuffer);
            mpu2.dmpGetGravity(&gravity2, &q2);
            mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
            roll2 = ypr2[2]*180/M_PI;
    }

        if (mpu3.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 



    
            // display Euler angles in degrees
            mpu3.dmpGetQuaternion(&q3, fifoBuffer);
            mpu3.dmpGetGravity(&gravity3, &q3);
            mpu3.dmpGetYawPitchRoll(ypr3, &q3, &gravity3);
            roll3 = ypr3[2]*180/M_PI;
    }

}