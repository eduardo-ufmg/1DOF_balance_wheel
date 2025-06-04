#include "PID.hpp"

#include <Arduino.h> // For constrain, fabs

PID::PID(float Kp, float Ki, float Kd, float outputMin, float outputMax)
    : _Kp(Kp), _Ki(Ki), _Kd(Kd), _outputMin(outputMin), _outputMax(outputMax),
      _integralTerm(0.0f), _previousError(0.0f), _previousInput(0.0f),
      _firstRun(true) {}

float PID::compute(float setpoint, float input, float dt) {
  if (dt <= 0.0f)
    return _outputMin; // Or some default safe output

  float error = setpoint - input;

  // Proportional term
  float P_out = _Kp * error;

  // Integral term (with anti-windup)
  _integralTerm += error * dt;
  // Anti-windup: clamp integral term if output is already saturated
  // This simple clamping might not be enough for strong windup, consider more
  // advanced anti-windup
  if (_integralTerm * _Ki > _outputMax)
    _integralTerm = _outputMax / _Ki;
  else if (_integralTerm * _Ki < _outputMin)
    _integralTerm = _outputMin / _Ki;
  float I_out = _Ki * _integralTerm;

  // Derivative term (on measurement to reduce setpoint derivative kick)
  float derivative;
  if (_firstRun) {
    derivative = 0;
    _firstRun = false;
  } else {
    derivative = (input - _previousInput) / dt;
  }
  float D_out =
      -_Kd * derivative; // Negative Kd because derivative is on measurement

  // Total output
  float output = P_out + I_out + D_out;

  // Store values for next iteration
  _previousError = error;
  _previousInput = input;

  // Clamp output
  return constrain(output, _outputMin, _outputMax);
}

void PID::setTunings(float Kp, float Ki, float Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}

void PID::setOutputLimits(float min, float max) {
  _outputMin = min;
  _outputMax = max;
  // Re-clamp integral term if limits change
  if (_integralTerm * _Ki > _outputMax)
    _integralTerm = _outputMax / _Ki;
  else if (_integralTerm * _Ki < _outputMin)
    _integralTerm = _outputMin / _Ki;
}

void PID::reset() {
  _integralTerm = 0.0f;
  _previousError = 0.0f;
  _previousInput = 0.0f; // Or current input if available
  _firstRun = true;
}
