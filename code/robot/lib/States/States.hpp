#ifndef STATES_HPP
#define STATES_HPP

#include "Encoder.hpp"
#include "Estimator.hpp"
#include "Sensor.hpp"

class States
{
public:
    States(Sensor & imu, Encoder & flywheel_encoder, Estimator & estimator);

    // Public members to hold the state vector
    float phi;     // Body tilt angle (rad)
    float phi_dot; // Body tilt angular velocity (rad/s)
    float psi;     // Flywheel angle relative to body (rad)
    float psi_dot; // Flywheel angular velocity (rad/s)

    // Updates all state variables based on new sensor readings
    void update(float dt);

private:
    // References to the hardware and estimator components
    Sensor & _imu;
    Encoder & _flywheel_encoder;
    Estimator & _estimator;

    // Internal variable to track the previous wheel angle for velocity calculation
    float _last_psi;
};

#endif // STATES_HPP