#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    Q_angle = 0.001f;  // Process noise variance for the angle
    Q_bias = 0.003f;   // Process noise variance for the gyroscope bias
    R_measure = 0.03f; // Measurement noise variance
    angle = 0.0f;
    bias = 0.0f;
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
}

void KalmanFilter::setAngle(float newAngle)
{
    angle = newAngle;
}

float KalmanFilter::getAngle(float newAngle, float newRate, float dt)
{
    // Predict
    angle += (newRate - bias) * dt;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    float K_0 = P[0][0] / (P[0][0] + R_measure);
    float K_1 = P[1][0] / (P[0][0] + R_measure);

    float y = newAngle - angle; // Angle residual

    angle += K_0 * y;
    bias += K_1 * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K_0 * P00_temp;
    P[0][1] -= K_0 * P01_temp;
    P[1][0] -= K_1 * P00_temp;
    P[1][1] -= K_1 * P01_temp;

    return angle;
}
