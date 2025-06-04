#include "StateSpaceController.hpp"

StateSpaceController::StateSpaceController(const float gains[STATE_DIMENSION]) {
  setGains(gains);
}

float StateSpaceController::compute(const float stateVector[STATE_DIMENSION]) {
  float controlOutput = 0.0f;
  // u = -K * x
  for (int i = 0; i < STATE_DIMENSION; ++i) {
    controlOutput += _K[i] * stateVector[i];
  }
  return -controlOutput; // Control law is u = -Kx
}

void StateSpaceController::setGains(const float gains[STATE_DIMENSION]) {
  for (int i = 0; i < STATE_DIMENSION; ++i) {
    _K[i] = gains[i];
  }
}
