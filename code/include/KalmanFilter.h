#pragma once

class KalmanFilter
{
public:
    KalmanFilter();
    void setAngle(float angle);
    float getAngle(float newAngle, float newRate, float dt);

private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias, rate;
    float P[2][2];
};
