#include "Sensor.hpp"

Sensor::Sensor()
{
    // The constructor is intentionally left empty.
    // Hardware initialization is handled in the begin() method.
}

bool Sensor::begin()
{
    if (!_mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        return false;
    }

    // Set accelerometer range
    _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

    // Set gyroscope range
    _mpu.setGyroRange(MPU6050_RANGE_500_DEG);

    // Set filter bandwidth
    _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 Found!");
    return true;
}

void Sensor::read()
{
    // Get new sensor events with the readings
    _mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
}