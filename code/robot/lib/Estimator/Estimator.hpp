#ifndef ESTIMATOR_HPP
#define ESTIMATOR_HPP

class Estimator
{
public:
    // Constructor: Initializes the filter's parameters
    Estimator(float q_angle, float q_bias, float r_measure);

    // Computes the estimated angle
    // Takes dt (time since last update in seconds), gyro rate (in rad/s), and accel angle (in rad)
    float update(float dt, float gyro_rate, float accel_angle);

    // Getter for the current angle estimate
    float getAngle() const;

private:
    // Kalman filter parameters
    float _q_angle;   // Process noise variance for the angle
    float _q_bias;    // Process noise variance for the gyro bias
    float _r_measure; // Measurement noise variance

    // Filter state variables
    float _angle;     // The angle estimate
    float _bias;      // The gyro bias estimate
    float _ECM[2][2]; // The error covariance matrix
};

#endif // ESTIMATOR_HPP