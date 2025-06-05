#ifndef MPU6050_INTERFACE_H
#define MPU6050_INTERFACE_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class MPU6050Interface {
public:
  MPU6050Interface();
  bool begin();
  bool read(float &accel, float &gyro);

private:
  Adafruit_MPU6050 mpu;
};

#endif // MPU6050_INTERFACE_H
