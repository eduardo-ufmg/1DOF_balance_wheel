#include "IMU.h"
#include <Arduino.h> // Required for millis() and other Arduino functions

IMU::IMU(uint8_t address) : addr(address), angle(0), rate(0)
{ // Initialize rate and KalmanFilter
    lastUpdateTime = millis();
}

bool IMU::begin()
{
    if (!mpu.begin(addr))
        return false;

    // Configure MPU6050 as needed
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    // You might want to calibrate the IMU or set an initial angle for the Kalman
    // filter
    float initialAccelAngle = 0; // Calculate initial angle from first readings
    kalmanFilter.setAngle(initialAccelAngle);
    return true;
}

void IMU::update()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate delta time (dt) for the Kalman filter
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0f; // Convert ms to seconds
    lastUpdateTime = currentTime;

    // For X-axis angle (Roll):
    // Accelerometer angle is calculated from Y and Z acceleration.
    // Gyroscope rate is around the X-axis.

    // Calculate accelerometer angle (roll)
    // atan2(Y, Z) gives the angle relative to the YZ plane.
    // Use radians for atan2
    float accelRoll = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x +
                                                   a.acceleration.z * a.acceleration.z));

    // Get gyroscope rate around the X-axis (roll rate)
    float gyroRateX = g.gyro.x;

    // Pass accelerometer angle and gyroscope rate to the Kalman filter
    angle = kalmanFilter.getAngle(accelRoll, gyroRateX, dt);
    rate = gyroRateX; // Store the current angular rate for potential use
}

float IMU::getAngle()
{
    return angle;
}

float IMU::getRate()
{
    return rate;
} // New method to expose angular rate
