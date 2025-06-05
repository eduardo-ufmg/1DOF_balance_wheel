#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() : angle(0), bias(0), rate(0) {
  P[0][0] = 0;
  P[0][1] = 0;
  P[1][0] = 0;
  P[1][1] = 0;
}

void KalmanFilter::initialize(float q_angle, float q_bias, float r_measure) {
  Q_angle = q_angle;
  Q_bias = q_bias;
  R_measure = r_measure;
}

float KalmanFilter::update(float newAngle, float newRate, float dt) {
  // Kalman filter implementation
  // ...implementation to be filled...
  return angle;
}
