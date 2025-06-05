#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
  PIDController(float kp, float ki, float kd);
  void setTunings(float kp, float ki, float kd);
  float compute(float setpoint, float input, float dt);
  void reset();

private:
  float kp, ki, kd;
  float prevError, integral;
};

#endif // PID_CONTROLLER_H
