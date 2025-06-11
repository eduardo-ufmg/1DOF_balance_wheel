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
    void calibrate();
    float getAngle();
    float getRate();

private:
    Adafruit_MPU6050 mpu;
    uint8_t addr;
    float angle;
    float rate;
    KalmanFilter kalmanFilter;
    unsigned long lastUpdateTime;
};