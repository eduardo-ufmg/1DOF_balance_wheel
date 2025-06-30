#include "States.hpp"
#include <math.h>

States::States(Sensor & imu, Encoder & flywheel_encoder, Estimator & estimator)
    : _imu(imu), _flywheel_encoder(flywheel_encoder), _estimator(estimator), phi(0.0f),
      phi_dot(0.0f), psi(0.0f), psi_dot(0.0f), _last_psi(0.0f)
{
    // Constructor initializes all state variables and references
}

void States::update(float dt)
{
    // 1. Read the raw sensor data
    _imu.read();
    psi = _flywheel_encoder.getAngle(); // Get angle from the single flywheel encoder

    // 2. Compute Flywheel State (psi_dot)
    psi_dot = (psi - _last_psi) / dt;
    _last_psi = psi; // Update last known flywheel angle for the next derivative calculation

    // 3. Compute Body States (phi and phi_dot) for ROLL balancing
    float accel_x = _imu.accelEvent.acceleration.x;
    float accel_y = _imu.accelEvent.acceleration.y;
    float accel_angle = atan2(accel_y, accel_x); // Angle from accelerometer

    phi_dot = _imu.gyroEvent.gyro.z; // Angular velocity from gyroscope (roll)

    // Use the Kalman filter to get the fused angle for the body tilt
    phi = _estimator.update(dt, phi_dot, accel_angle);
}