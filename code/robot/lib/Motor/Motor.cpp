#include "Motor.hpp"
#include <Arduino.h>

// Constructor implementation
Motor::Motor(uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin, uint8_t pwmChannel,
             uint32_t pwmFrequency, uint8_t pwmResolution)
    : _pwmPin(pwmPin), _dirPin(dirPin), _brakePin(brakePin), _pwmChannel(pwmChannel),
      _pwmFrequency(pwmFrequency), _pwmResolution(pwmResolution)
{
    // Calculate the maximum duty cycle value based on the resolution
    _maxDutyCycle = (1 << _pwmResolution) - 1;
}

// Initialization implementation
void Motor::begin()
{
    // Configure the ESP32 LEDC (PWM) peripheral
    ledcSetup(_pwmChannel, _pwmFrequency, _pwmResolution);

    // Attach the PWM pin to the configured channel
    ledcAttachPin(_pwmPin, _pwmChannel);

    // Set pin modes for direction and brake pins
    pinMode(_dirPin, OUTPUT);
    pinMode(_brakePin, OUTPUT);

    // Release the brake by default
    releaseBrake();
}

// Set speed implementation
void Motor::setSpeed(float speed)
{
    // Constrain speed to the range -1.0 to 1.0
    speed = constrain(speed, -1.0, 1.0);

    // Set direction based on the sign of the speed
    // HIGH for forward, LOW for reverse (this might need to be tuned)
    digitalWrite(_dirPin, speed >= 0 ? HIGH : LOW);

    // Calculate the duty cycle for the active-low PWM signal.
    // A speed of 1.0 (max forward) corresponds to a duty cycle of 0.
    // A speed of 0.0 (stop) corresponds to a duty cycle of _maxDutyCycle.
    uint32_t dutyCycle = _maxDutyCycle * (1.0 - abs(speed));

    // Write the duty cycle value to the PWM channel
    ledcWrite(_pwmChannel, dutyCycle);
}

// Brake implementation
void Motor::brake()
{
    // Brake is active low, so write LOW to engage it.
    digitalWrite(_brakePin, LOW);
}

// Release brake implementation
void Motor::releaseBrake()
{
    // Brake is active low, so write HIGH to disengage it.
    digitalWrite(_brakePin, HIGH);
}