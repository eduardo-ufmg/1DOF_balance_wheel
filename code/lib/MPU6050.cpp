#include "MPU6050.hpp"

#include "Config.hpp" // For MPU6050_ADDRESS

MPU6050::MPU6050() {
  // Constructor
}

bool MPU6050::begin() {
  if (!mpu.begin(MPU6050_ADDRESS)) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // +/- 8G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // +/- 500 deg/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // 21 Hz bandwidth

  return true;
}

bool MPU6050::update() {
  return mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
}

float MPU6050::getAccelX_mss() const { return accelEvent.acceleration.x; }

float MPU6050::getAccelY_mss() const { return accelEvent.acceleration.y; }

float MPU6050::getAccelZ_mss() const { return accelEvent.acceleration.z; }

float MPU6050::getGyroX_rads() const {
  return gyroEvent.gyro.x; // Adafruit library provides gyro data in rad/s
}

float MPU6050::getGyroY_rads() const { return gyroEvent.gyro.y; }

float MPU6050::getGyroZ_rads() const { return gyroEvent.gyro.z; }

float MPU6050::getAngleX_rad_from_accel() const {
  // Angle around X-axis (pitch) using Y and Z accelerometer components
  // atan2 is generally preferred over atan for full quadrant coverage
  return atan2(accelEvent.acceleration.y, accelEvent.acceleration.z);
}
