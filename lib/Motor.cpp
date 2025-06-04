#include "Motor.hpp"

#include <Arduino.h>  // For HIGH, LOW, digitalWrite, etc.

#include "Config.hpp"  // For MOTOR_MAX_PWM_DUTY

Motor::Motor(int pwmPin, int dirPin, int brakePin, int encAPin, int encBPin,
             int pwmChannel, int pwmFrequency, int pwmResolution,
             float encoderPPR)
    : _pwmPin(pwmPin),
      _dirPin(dirPin),
      _brakePin(brakePin),
      _encAPin(encAPin),
      _encBPin(encBPin),
      _pwmChannel(pwmChannel),
      _pwmFrequency(pwmFrequency),
      _pwmResolution(pwmResolution),
      _encoderPPR(encoderPPR),
      _lastEncoderCount(0),
      _currentSpeed_rad_s(0.0f),
      _previousSpeed_rad_s(0.0f),
      _currentAcceleration_rad_ss(0.0f),
      _brakeActive(false) {
  _maxDutyCycle = (1 << _pwmResolution) - 1;
  if (_encoderPPR > 0) {
    _radsPerCount = (2.0f * PI) / _encoderPPR;
  } else {
    _radsPerCount = 0.0f;  // Avoid division by zero
  }
  // Note: ESP32Encoder uses PCNT internally, which counts on 4 edges for
  // quadrature. The library handles this, so PPR should be the pulses per
  // revolution of the encoder disk itself. If your PPR value is for "counts per
  // revolution" after quadrature, adjust accordingly or test. For now, assuming
  // _encoderPPR is the fundamental pulses per one shaft revolution.
  // ESP32Encoder default is X4 encoding.
}

void Motor::begin() {
  // Setup PWM
  ledcSetup(_pwmChannel, _pwmFrequency, _pwmResolution);
  ledcAttachPin(_pwmPin, _pwmChannel);
  ledcWrite(_pwmChannel, 0);  // Start with motor off

  // Setup Direction Pin
  pinMode(_dirPin, OUTPUT);
  digitalWrite(_dirPin, LOW);  // Default direction

  // Setup Brake Pin
  pinMode(_brakePin, OUTPUT);
  digitalWrite(_brakePin, LOW);  // Assuming LOW releases brake, HIGH activates
  _brakeActive = false;

  // Setup Encoder
  ESP32Encoder::useInternalWeakPullResistors =
      UP;  // Or DOWN, or NONE if external resistors are used
  _encoder.attachHalfQuad(
      _encAPin,
      _encBPin);  // Use attachHalfQuad or attachFullQuad as per wiring
  _encoder.clearCount();
  _lastEncoderCount = _encoder.getCount();
}

void Motor::update(float dt) {
  if (dt <= 0.0f) return;  // Avoid division by zero or negative dt

  long currentEncoderCount = _encoder.getCount();
  long countDifference = currentEncoderCount - _lastEncoderCount;
  _lastEncoderCount = currentEncoderCount;

  // Speed calculation
  _previousSpeed_rad_s = _currentSpeed_rad_s;
  _currentSpeed_rad_s = (countDifference * _radsPerCount) / dt;

  // Acceleration calculation (simple derivative, can be noisy)
  _currentAcceleration_rad_ss =
      (_currentSpeed_rad_s - _previousSpeed_rad_s) / dt;
}

void Motor::setEffort(float effort) {
  if (_brakeActive) return;  // Do not run motor if brake is active

  // Clamp effort to [-1.0, 1.0]
  if (effort > 1.0f) effort = 1.0f;
  if (effort < -1.0f) effort = -1.0f;

  // Determine direction
  // Assuming HIGH on _dirPin is forward, LOW is reverse. Adjust if needed.
  if (effort >= 0) {
    digitalWrite(_dirPin, HIGH);  // Forward
  } else {
    digitalWrite(_dirPin, LOW);  // Reverse
  }

  // Calculate PWM duty cycle
  int dutyCycle = (int)(abs(effort) * _maxDutyCycle);
  ledcWrite(_pwmChannel, dutyCycle);
}

void Motor::activateBrake() {
  ledcWrite(_pwmChannel, 0);      // Stop PWM
  digitalWrite(_brakePin, HIGH);  // Activate brake (assuming active HIGH)
  _brakeActive = true;
  _currentSpeed_rad_s = 0.0f;  // Assume brake stops motion quickly
  _currentAcceleration_rad_ss = 0.0f;
}

void Motor::releaseBrake() {
  digitalWrite(_brakePin, LOW);  // Release brake
  _brakeActive = false;
}

void Motor::stop() {
  setEffort(0.0f);
  activateBrake();  // Or just setEffort(0) if brake is not always desired on
                    // stop
}

float Motor::getSpeed_rad_s() const { return _currentSpeed_rad_s; }

float Motor::getAcceleration_rad_ss() const {
  return _currentAcceleration_rad_ss;
}

long Motor::getEncoderCounts() const { return _encoder.getCount(); }

float Motor::getDisplacement_rad() const {
  return _encoder.getCount() * _radsPerCount;
}
