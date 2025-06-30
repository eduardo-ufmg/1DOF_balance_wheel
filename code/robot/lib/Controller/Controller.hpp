#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Arduino.h> // For basic types

class Controller
{
public:
    // Constructor: Initializes the controller with the feedback gain K
    // The gain array must have 4 elements.
    Controller(const float k_gains[4]);

    // Computes the control signal (motor output) based on the current state
    // State array: [phi, phi_dot, psi, psi_dot]
    // phi: Body angle (rad)
    // phi_dot: Body angular velocity (rad/s)
    // psi: Wheel angle (rad)
    // psi_dot: Wheel angular velocity (rad/s)
    float compute(const float state[4]);

private:
    // State-feedback gain matrix K
    float _K[4];
};

#endif // CONTROLLER_HPP