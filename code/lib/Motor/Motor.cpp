#include "Motor.h"
#include <Arduino.h>

// Define a static pointer to the Motor object to allow the ISR to access it.
// This is a common pattern for attaching member functions as ISRs in Arduino.
static Motor * _motorInstance;

// Encoder ISR function
void IRAM_ATTR encoderISR()
{
    if (_motorInstance) {
        _motorInstance->updateEncoder();
    }
}

Motor::Motor(int brakePin, int pwmPin, int dirPin, int encaPin, int encbPin)
    : _brakePin(brakePin), _pwmPin(pwmPin), _dirPin(dirPin), _encaPin(encaPin), _encbPin(encbPin),
      _encoderCount(0)
{
    // Store the current instance to be accessed by the static ISR
    _motorInstance = this;
}

void Motor::begin()
{
    pinMode(_brakePin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_encaPin, INPUT_PULLUP); // Encoder A pin
    pinMode(_encbPin, INPUT_PULLUP); // Encoder B pin

    // Attach interrupt to encoder A pin.
    // CHANGE detects any change in the pin state (HIGH to LOW or LOW to HIGH).
    attachInterrupt(digitalPinToInterrupt(_encaPin), encoderISR, CHANGE);
}

void Motor::setPWM(int pwm)
{
    // Ensure PWM value is within the 0-255 range for analogWrite
    pwm = constrain(pwm, -255, 255);
    analogWrite(_pwmPin, abs(pwm));
    digitalWrite(_dirPin, pwm >= 0 ? HIGH : LOW);
}

void Motor::brake(bool on)
{
    digitalWrite(_brakePin, on ? HIGH : LOW);
}

long Motor::getEncoderCount()
{
    return _encoderCount;
}

// Encoder ISR implementation (called by the static encoderISR function)
void Motor::updateEncoder()
{
    // Read both encoder pins
    int enca_state = digitalRead(_encaPin);
    int encb_state = digitalRead(_encbPin);

    // Determine direction and update encoder count
    // This is a common method for quadrature encoders
    if (enca_state == HIGH) {
        if (encb_state == HIGH) {
            _encoderCount--; // Clockwise or Counter-clockwise, depending on wiring
        } else {
            _encoderCount++; // Opposite direction
        }
    } else {
        if (encb_state == HIGH) {
            _encoderCount++; // Opposite direction
        } else {
            _encoderCount--; // Clockwise or Counter-clockwise
        }
    }
}