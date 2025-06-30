// Implementation for Motor.hpp
#include "Motor.hpp"

Motor::Motor(int pwmPin, int dirPin, int brakePin, int pwmChannel, int freq, int resolution)
    : _pwmPin(pwmPin), _dirPin(dirPin), _brakePin(brakePin), _pwmChannel(pwmChannel), _freq(freq),
      _resolution(resolution)
{
    _maxDutyCycle = (1 << _resolution) - 1;
    pinMode(_dirPin, OUTPUT);
    pinMode(_brakePin, OUTPUT);
    digitalWrite(_brakePin, LOW); // Engage brake
    ledcSetup(_pwmChannel, _freq, _resolution);
    ledcAttachPin(_pwmPin, _pwmChannel);
}

void Motor::setSpeed(float speed)
{

    speed = constrain(speed, -1.0f, 1.0f); // Ensure speed is within [-1, 1]
    int signedDutyCycle = static_cast<int>(speed * _maxDutyCycle);

    // This motor's duty cycle is inverted, meaning that the higher the time
    // it is off, the faster it goes.

    if (signedDutyCycle > 0) {
        ledcWrite(_pwmChannel, _maxDutyCycle - signedDutyCycle);
        digitalWrite(_dirPin, HIGH); // Forward direction
    } else if (signedDutyCycle < 0) {
        ledcWrite(_pwmChannel, _maxDutyCycle + signedDutyCycle);
        digitalWrite(_dirPin, LOW); // Reverse direction
    } else {
        ledcWrite(_pwmChannel, _maxDutyCycle); // Stop
    }
}

void Motor::brake()
{
    digitalWrite(_brakePin, LOW);
}

void Motor::releaseBrake()
{
    digitalWrite(_brakePin, HIGH);
}