#ifndef MPU6050_HPP
#define MPU6050_HPP

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class MPU6050 {
 public:
  MPU6050();
  bool begin();
  bool update();  // Call this frequently to get new sensor events

  // Get raw sensor data (scaled)
  float getAccelX_mss() const;  // m/s^2
  float getAccelY_mss() const;  // m/s^2
  float getAccelZ_mss() const;  // m/s^2
  float getGyroX_rads() const;  // rad/s
  float getGyroY_rads() const;  // rad/s
  float getGyroZ_rads() const;  // rad/s

  // Get calculated angle from accelerometer for the X-axis tilt
  float getAngleX_rad_from_accel() const;

 private:
  Adafruit_MPU6050 mpu;
  sensors_event_t accelEvent;
  sensors_event_t gyroEvent;
  sensors_event_t tempEvent;  // Temperature data, not used directly but read
};

#endif  // MPU6050_HPP
