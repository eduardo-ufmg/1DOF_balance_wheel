#include "KalmanFilter.hpp"

#include <math.h>  // For fabs

KalmanFilter::KalmanFilter(float Q_angle_val, float Q_bias_val,
                           float R_measure_val)
    : Q_angle(Q_angle_val),
      Q_bias(Q_bias_val),
      R_measure(R_measure_val),
      angle(0.0f),
      bias(0.0f),
      rate(0.0f) {
  // Initialize covariance matrix
  P[0][0] = 0.0f;
  P[0][1] = 0.0f;
  P[1][0] = 0.0f;
  P[1][1] = 0.0f;
}

float KalmanFilter::update(float newAngle, float newRate, float dt) {
  // Prediction step
  // Rate = newRate - bias
  rate = newRate - bias;
  // Angle = angle + dt * rate
  angle += dt * rate;

  // Update error covariance matrix
  // P_dot = A * P + P * A_T + Q
  // Here, A = [[0, -dt], [0, 0]] (linearized system for angle and bias)
  // Simplified:
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  // Measurement update (Kalman gain and correction)
  // Innovation (measurement residual)
  float y = newAngle - angle;

  // Innovation covariance
  float S = P[0][0] + R_measure;

  // Kalman gain
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Update angle and bias
  angle += K[0] * y;
  bias += K[1] * y;

  // Update error covariance matrix
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}
