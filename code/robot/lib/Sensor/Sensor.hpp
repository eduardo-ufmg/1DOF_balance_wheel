#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

class Sensor
{
public:
    // Public members to hold the sensor data
    sensors_event_t accelEvent;
    sensors_event_t gyroEvent;
    sensors_event_t tempEvent;

    // Constructor can be empty, initialization is done in begin()
    Sensor();

    // Initializes the sensor
    bool begin();

    // Reads the latest data from the sensor
    void read();

private:
    // The MPU6050 object from the Adafruit library
    Adafruit_MPU6050 _mpu;
};

#endif // SENSOR_HPP