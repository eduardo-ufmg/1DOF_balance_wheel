#include "Controller.hpp"

Controller::Controller(const float k_gains[4])
{
    // Copy the provided gains into the private member variable
    for (int i = 0; i < 4; ++i) {
        _K[i] = k_gains[i];
    }
}

float Controller::compute(const float state[4])
{
    // The control law is u = -K*x
    // where u is the control signal and x is the state vector.
    // This is a simple dot product.

    float control_signal = 0.0f;
    for (int i = 0; i < 4; ++i) {
        control_signal += _K[i] * state[i];
    }

    // The simulation calculates u = -K*x, so we return the negated result
    return -control_signal;
}