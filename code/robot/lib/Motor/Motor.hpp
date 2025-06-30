#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

class Motor
{
public:
    Motor(int pwmPin, int dirPin, int brakePin, int pwmChannel, int freq = 20000,
          int resolution = 8);
    void setSpeed(float speed);
    void brake();
    void releaseBrake();

private:
    int _pwmPin;
    int _dirPin;
    int _brakePin;
    int _pwmChannel;
    int _freq;
    int _resolution;
    int _maxDutyCycle;
};

#endif // MOTOR_HPP