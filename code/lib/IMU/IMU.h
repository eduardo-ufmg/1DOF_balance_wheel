#pragma once
#include "KalmanFilter.h"
#include <Adafruit_MPU6050.h>
#include <Wire.h>

class IMU
{
public:
    IMU(uint8_t address = 0x68);
    bool begin();
    void update();
    float getAngle(); // This will return the Kalman filtered angle
    float getRate();  // Get the current angular rate for debugging
                      // or other uses

private:
    Adafruit_MPU6050 mpu;
    uint8_t addr;
    float angle;                  // This will store the Kalman filtered angle
    float rate;                   // Store the current gyroscope rate
    KalmanFilter kalmanFilter;    // Add a KalmanFilter object
    unsigned long lastUpdateTime; // To calculate dt for Kalman filter
};
