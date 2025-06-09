#pragma once

class LQR
{
public:
    LQR();
    float compute(float angle, float rate, float wheel_speed);
    void setReference(float ref);

private:
    float ref;
    float K[3];
};