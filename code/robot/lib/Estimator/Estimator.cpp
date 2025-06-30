#include "Estimator.hpp"
#include <math.h>

Estimator::Estimator(float q_angle, float q_bias, float r_measure)
    : _q_angle(q_angle), _q_bias(q_bias), _r_measure(r_measure), _angle(0.0f), _bias(0.0f)
{
    // Initialize the error covariance matrix
    _ECM[0][0] = 1.0f;
    _ECM[0][1] = 0.0f;
    _ECM[1][0] = 0.0f;
    _ECM[1][1] = 1.0f;
}

float Estimator::update(float dt, float gyro_rate, float accel_angle)
{
    // ---- Prediction Step ----
    // Predict the state
    _angle += dt * (gyro_rate - _bias);

    // Predict the error covariance
    _ECM[0][0] += dt * (dt * _ECM[1][1] - _ECM[0][1] - _ECM[1][0] + _q_angle);
    _ECM[0][1] -= dt * _ECM[1][1];
    _ECM[1][0] -= dt * _ECM[1][1];
    _ECM[1][1] += _q_bias * dt;

    // ---- Update Step ----
    // Calculate the innovation
    float y = accel_angle - _angle;

    // Calculate the innovation covariance
    float S = _ECM[0][0] + _r_measure;

    // Calculate the Kalman gain
    float K[2];
    K[0] = _ECM[0][0] / S;
    K[1] = _ECM[1][0] / S;

    // Update the state estimate
    _angle += K[0] * y;
    _bias += K[1] * y;

    // Update the error covariance
    float P00_temp = _ECM[0][0];
    float P01_temp = _ECM[0][1];

    _ECM[0][0] -= K[0] * P00_temp;
    _ECM[0][1] -= K[0] * P01_temp;
    _ECM[1][0] -= K[1] * P00_temp;
    _ECM[1][1] -= K[1] * P01_temp;

    return _angle;
}

float Estimator::getAngle() const
{
    return _angle;
}