#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), prevError(0), integral(0) {}

void PIDController::setTunings(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

float PIDController::compute(float setpoint, float input, float dt) {
  float error = setpoint - input;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;
  return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset() {
  prevError = 0;
  integral = 0;
}
