#pragma once

class Controller
{
public:
    Controller();

    /**
     * @brief Sets the reference angle for the controller.
     * @param reference The desired angle in radians (0 is upright).
     */
    void setReference(float reference);

    /**
     * @brief Computes the control signal based on the current state.
     * @param angle The current angle of the robot (radians).
     * @param rate The current angular rate of the robot (rad/s).
     * @param wheel_position The current position of the wheel (radians).
     * @param wheel_speed The current speed of the wheel (rad/s).
     * @return The computed control signal.
     */
    float compute(float angle, float rate, float wheel_position, float wheel_speed);

private:
    float _reference_angle;

    // Discrete-time LQR gain K_d
    // Assumes state vector: [angle, rate, wheel_position, wheel_speed]
    const float K_d[4] = {-8.35843438e+00, -1.07745285e+00, 8.18723553e-18, -3.10273526e-02};
};
