#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp), ki(ki), kd(kd), prevError(0), integral(0)
{
}

float PIDController::compute(float setpoint, float measured, float dt)
{
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;
    return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset()
{
    prevError = 0;
    integral = 0;
}
