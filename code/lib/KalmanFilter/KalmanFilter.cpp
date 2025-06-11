#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    Q_angle = 0.001f;  // Process noise variance for the angle
    Q_bias = 0.003f;   // Process noise variance for the gyroscope bias
    R_measure = 0.03f; // Measurement noise variance
    angle = 0.0f;
    bias = 0.0f;

    // Initialize P with some uncertainty
    P[0][0] = 1.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 1.0f;
}

void KalmanFilter::setAngle(float newAngle)
{
    angle = newAngle;
}

float KalmanFilter::getAngle(float newAngle, float newRate, float dt)
{
    // Predict
    rate = newRate - bias;
    angle += dt * rate;

    // Update covariance
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}