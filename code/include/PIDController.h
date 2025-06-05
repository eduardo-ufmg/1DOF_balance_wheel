#pragma once

class PIDController {
public:
  PIDController(float kp, float ki, float kd);
  float compute(float setpoint, float measured, float dt);
  void reset();

private:
  float kp, ki, kd;
  float prevError, integral;
};
