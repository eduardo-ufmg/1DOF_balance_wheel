#include "Controller.h"

Controller::Controller() : _reference_angle(0.0f)
{
}

void Controller::setReference(float reference)
{
    _reference_angle = reference;
}

float Controller::compute(float angle, float rate, float wheel_position, float wheel_speed)
{
    // The LQR controller is a state feedback controller of the form u = -K*x,
    // where x is the state vector. The state is assumed to be regulated around the origin.
    // The state vector is assumed to be [angle, rate, wheel_position, wheel_speed].
    // We want to regulate the angle around the reference angle.
    float angle_error = angle - _reference_angle;

    // The control law is u = - (k1*angle_error + k2*rate + k3*wheel_position + k4*wheel_speed)
    // Note the negative sign because the gain K is typically defined for u = -Kx.
    float control_signal =
        -(K_d[0] * angle_error + K_d[1] * rate + K_d[2] * wheel_position + K_d[3] * wheel_speed);

    return control_signal;
}