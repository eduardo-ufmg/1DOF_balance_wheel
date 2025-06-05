#include "IMU.h"

IMU::IMU(uint8_t address) : addr(address), angle(0) {}

bool IMU::begin() {
  if (!mpu.begin(addr))
    return false;
  // Configure MPU6050 as needed
  return true;
}

void IMU::update() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // TODO: Use Kalman filter for angle
}

float IMU::getAngle() { return angle; }
