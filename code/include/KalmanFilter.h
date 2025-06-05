#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
  KalmanFilter();
  void initialize(float q_angle, float q_bias, float r_measure);
  float update(float newAngle, float newRate, float dt);

private:
  float angle, bias, rate;
  float P[2][2];
  float Q_angle, Q_bias, R_measure;
};

#endif // KALMAN_FILTER_H
