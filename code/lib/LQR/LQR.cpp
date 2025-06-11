#include "LQR.h"

LQR::LQR() : ref(0)
{
    K[0] = 7.44990783; // K for angle (theta)
    K[1] = 0.09897606; // K for angular velocity (theta_dot)
    K[2] = 1.20793109; // K for wheel angular velocity (phi_dot)
}

float LQR::compute(float angle, float rate, float wheel_speed)
{
    // State vector x = [angle, rate, wheel_speed]
    // Control law: u = -K * (x - x_ref)
    // Reference is for the angle only, x_ref = [ref, 0, 0]
    float u = -(K[0] * (angle - ref) + K[1] * rate + K[2] * wheel_speed);
    return u;
}

void LQR::setReference(float r)
{
    ref = r;
}