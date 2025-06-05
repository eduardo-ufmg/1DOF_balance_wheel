#include "StateSpaceController.h"

StateSpaceController::StateSpaceController()
    : A(nullptr), B(nullptr), C(nullptr), K(nullptr), n(0) {}

void StateSpaceController::setParameters(float *A, float *B, float *C, float *K,
                                         int n) {
  this->A = A;
  this->B = B;
  this->C = C;
  this->K = K;
  this->n = n;
}

float StateSpaceController::compute(float *state, float reference) {
  // State-space control law: u = -Kx + r
  float u = reference;
  for (int i = 0; i < n; ++i) {
    u -= K[i] * state[i];
  }
  return u;
}
