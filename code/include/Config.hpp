#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Arduino.h>

// --- Main Loop ---
#define LOOP_PERIOD_MS 10 // Target loop period in milliseconds (100Hz)

// --- MPU6050 IMU ---
#define MPU6050_ADDRESS 0x68
// Default I2C pins for WEMOS D1 R32 are SDA: GPIO21, SCL: GPIO22
// Wire.begin() will use default pins.

// --- Motor Nidec24H ---
// Control Pins
#define MOTOR_PIN_BRAKE 14
#define MOTOR_PIN_PWM 27
#define MOTOR_PIN_DIR 16
// Encoder Pins
#define MOTOR_PIN_ENCA 25
#define MOTOR_PIN_ENCB 26

// PWM Configuration
#define MOTOR_PWM_CHANNEL 0       // ESP32 LEDC PWM channel (0-15)
#define MOTOR_PWM_FREQUENCY 20000 // PWM frequency in Hz (e.g., 20kHz)
#define MOTOR_PWM_RESOLUTION 10 // PWM resolution in bits (e.g., 10 for 0-1023)
#define MOTOR_MAX_PWM_DUTY                                                     \
  ((1 << MOTOR_PWM_RESOLUTION) - 1) // Max PWM duty based on resolution

// Encoder Configuration
#define MOTOR_ENCODER_PPR (100.0f)

// --- Kalman Filter ---
// Tuning parameters for the Kalman filter
#define KALMAN_Q_ANGLE 0.001f  // Process noise variance for angle
#define KALMAN_Q_BIAS 0.003f   // Process noise variance for gyro bias
#define KALMAN_R_MEASURE 0.03f // Measurement noise variance

// --- PID Controller (for Motor Acceleration) ---
// IMPORTANT: These gains need to be tuned.
#define MOTOR_PID_KP 0.5f  // Proportional gain (e.g., 0.5f)
#define MOTOR_PID_KI 0.1f  // Integral gain (e.g., 0.1f)
#define MOTOR_PID_KD 0.01f // Derivative gain (e.g., 0.01f)
// PID output limits (representing motor effort, will be mapped to PWM)
#define MOTOR_PID_OUTPUT_MIN -1.0f // Min motor effort
#define MOTOR_PID_OUTPUT_MAX 1.0f  // Max motor effort

// --- State-Space Controller ---
// State vector: [body_angle (rad), body_angular_velocity (rad/s),
// wheel_angular_velocity (rad/s)]
#define STATE_DIMENSION 3
// IMPORTANT: These gains (K1, K2, K3) must be calculated from your system's
// model and tuned. u = - (K1*angle + K2*angle_velocity + K3*wheel_velocity) The
// output 'u' is the desired motor acceleration (rad/s^2).
#define K1_BODY_ANGLE -30.0f   // Gain for body angle (e.g., -30.0f)
#define K2_BODY_ANG_VEL -1.5f  // Gain for body angular velocity (e.g., -1.5f)
#define K3_WHEEL_ANG_VEL 0.05f // Gain for wheel angular velocity (e.g., 0.05f)
const float STATE_SPACE_GAINS[STATE_DIMENSION] = {
    K1_BODY_ANGLE, K2_BODY_ANG_VEL, K3_WHEEL_ANG_VEL};

// --- Physical Constants ---
// (Optional, if needed for model-based calculations or normalizations)
#define GRAVITY_ACCEL 9.80665f // m/s^2

#endif // CONFIG_HPP
