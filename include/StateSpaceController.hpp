#ifndef STATE_SPACE_CONTROLLER_HPP
#define STATE_SPACE_CONTROLLER_HPP

#include "Config.hpp"  // For STATE_DIMENSION

class StateSpaceController {
 public:
  StateSpaceController(const float gains[STATE_DIMENSION]);

  // Computes the control output (desired motor acceleration)
  // stateVector: [body_angle, body_angular_velocity, wheel_angular_velocity]
  float compute(const float stateVector[STATE_DIMENSION]);

  void setGains(const float gains[STATE_DIMENSION]);

 private:
  float _K[STATE_DIMENSION];  // Gain matrix (actually a vector here for SISO)
};

#endif  // STATE_SPACE_CONTROLLER_HPP
