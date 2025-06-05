#include "Motor.h"
#include <Arduino.h>

Motor::Motor(int brakePin, int pwmPin, int dirPin, int encaPin, int encbPin)
    : _brakePin(brakePin), _pwmPin(pwmPin), _dirPin(dirPin), _encaPin(encaPin),
      _encbPin(encbPin), _encoderCount(0) {}

void Motor::begin() {
  pinMode(_brakePin, OUTPUT);
  pinMode(_pwmPin, OUTPUT);
  pinMode(_dirPin, OUTPUT);
  pinMode(_encaPin, INPUT);
  pinMode(_encbPin, INPUT);
  // Attach interrupts for encoder as needed
}

void Motor::setPWM(int pwm) {
  analogWrite(_pwmPin, abs(pwm));
  digitalWrite(_dirPin, pwm >= 0 ? HIGH : LOW);
}

void Motor::brake(bool on) { digitalWrite(_brakePin, on ? HIGH : LOW); }

long Motor::getEncoderCount() { return _encoderCount; }

void Motor::updateEncoder() {
  // TODO: Implement encoder ISR
}
