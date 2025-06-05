#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
  Q_angle = 0.001f;
  Q_bias = 0.003f;
  R_measure = 0.03f;
  angle = 0.0f;
  bias = 0.0f;
  P[0][0] = 0;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 0;
}

void KalmanFilter::setAngle(float newAngle) { angle = newAngle; }

float KalmanFilter::getAngle(float newAngle, float newRate, float dt) {
  // TODO: Implement Kalman filter equations
  return angle;
}
