#ifndef KALMAN_HPP
#define KALMAN_HPP

class KalmanFilter {
public:
  KalmanFilter(float Q_angle, float Q_bias, float R_measure);

  // Updates the filter with new measurements
  // newAngle: angle from accelerometer (radians)
  // newRate: angular rate from gyroscope (radians/second)
  // dt: time delta in seconds
  float update(float newAngle, float newRate, float dt);

  float getAngle() const { return angle; }
  float getRate() const { return rate; } // Estimated rate (bias corrected)
  float getBias() const { return bias; }

private:
  // Kalman filter parameters
  float Q_angle;   // Process noise variance for the accelerometer
  float Q_bias;    // Process noise variance for the gyro bias
  float R_measure; // Measurement noise variance

  // Filter state variables
  float angle; // The angle output from the filter (radians)
  float bias;  // The gyro bias output from the filter (radians/second)
  float rate;  // Estimated rate (newRate - bias)

  // Covariance matrix
  float P[2][2];
};

#endif // KALMAN_HPP
