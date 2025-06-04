#ifndef PID_HPP
#define PID_HPP

class PID {
 public:
  PID(float Kp, float Ki, float Kd, float outputMin, float outputMax);

  float compute(float setpoint, float input, float dt);
  void setTunings(float Kp, float Ki, float Kd);
  void setOutputLimits(float min, float max);
  void reset();

 private:
  float _Kp, _Ki, _Kd;
  float _outputMin, _outputMax;
  float _integralTerm;
  float _previousError;
  float _previousInput;  // For derivative on measurement
  bool _firstRun;
};

#endif  // PID_HPP
