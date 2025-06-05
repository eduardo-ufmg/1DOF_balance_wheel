#include "MPU6050Interface.h"

MPU6050Interface::MPU6050Interface() {}

bool MPU6050Interface::begin() {
  Wire.begin();
  return mpu.begin(0x68);
}

bool MPU6050Interface::read(float &accel, float &gyro) {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp))
    return false;
  accel = a.acceleration.y; // Adjust axis as needed
  gyro = g.gyro.y;          // Adjust axis as needed
  return true;
}
