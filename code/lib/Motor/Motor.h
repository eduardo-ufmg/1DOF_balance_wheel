#pragma once

#include <ESP32Encoder.h>

class Motor
{
public:
    Motor(int brakePin, int pwmPin, int dirPin, int encaPin, int encbPin, int ticksPerRev);
    void begin();
    void setPWM(int pwm);
    void brake(bool on);
    long getEncoderCount();
    long clearEncoderCount();
    int getTicksPerRev();

private:
    int _brakePin, _pwmPin, _dirPin, _encaPin, _encbPin, _ticksPerRev;

    int _pwmChannel;
    int _pwmResolutionBits;
    int _pwmFrequency;

    ESP32Encoder _encoder;
};