#include "IMU.h"
#include <Arduino.h>

IMU::IMU(uint8_t address) : addr(address), angle(0), rate(0)
{
    lastUpdateTime = millis();
}

bool IMU::begin()
{
    if (!mpu.begin(addr))
        return false;

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    // Get an initial angle estimate from the accelerometer
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float initialAccelAngle = atan2(a.acceleration.y, a.acceleration.z);
    kalmanFilter.setAngle(initialAccelAngle);

    lastUpdateTime = millis();
    return true;
}

void IMU::calibrate()
{
    const int num_readings = 1000;
    for (int i = 0; i < num_readings; i++) {
        update();
        delay(5); // Small delay between readings
    }
}

void IMU::update()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f;
    lastUpdateTime = currentTime;

    float accelRoll = atan2(a.acceleration.y, a.acceleration.z);
    float gyroRateX = g.gyro.x;

    angle = kalmanFilter.getAngle(accelRoll, gyroRateX, dt);
    rate = gyroRateX;
}

float IMU::getAngle()
{
    return angle;
}

float IMU::getRate()
{
    return rate;
}