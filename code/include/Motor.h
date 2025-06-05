#pragma once

class Motor
{
public:
    Motor(int brakePin, int pwmPin, int dirPin, int encaPin, int encbPin);
    void begin();
    void setPWM(int pwm);
    void brake(bool on);
    long getEncoderCount();
    void updateEncoder();

private:
    int _brakePin, _pwmPin, _dirPin, _encaPin, _encbPin;
    volatile long _encoderCount;
};
