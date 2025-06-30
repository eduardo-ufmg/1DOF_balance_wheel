#include "Controller.hpp"
#include "Encoder.hpp"
#include "Estimator.hpp"
#include "Motor.hpp"
#include "Sensor.hpp"
#include "States.hpp"
#include <Arduino.h>

// --- Direction Factors so the System Matches the Model ---
#define PHI_DIR_FACTOR +1.0f // Body angle (phi) direction factor
#define PSI_DIR_FACTOR +1.0f // Flywheel angle (psi) direction factor
#define PWM_DIR_FACTOR +1.0f // PWM direction factor for the flywheel motor

// --- Pinout Definitions ---
#define FLYWHEEL_PWM_PIN 27
#define FLYWHEEL_DIR_PIN 16
#define FLYWHEEL_BRAKE_PIN 14

#define FLYWHEEL_ENC_A_PIN 25
#define FLYWHEEL_ENC_B_PIN 26

// --- Physical & Control Parameters ---
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define FLYWHEEL_PWM_CHANNEL 0

#define ENCODER_PPR 400

// --- LQR Controller Gains ---
const float K_GAINS[4] = {
    -2.93f, // K1 for body angle (phi)
    -0.35f, // K2 for body angular velocity (phi_dot)
    -0.01f, // K3 for flywheel angle (psi)
    -0.01f  // K4 for flywheel angular velocity (psi_dot)
};

// --- Kalman Filter Tuning Parameters ---
#define KALMAN_Q_ANGLE 0.001f
#define KALMAN_Q_BIAS 0.003f
#define KALMAN_R_MEASURE 0.03f

// --- Global Objects ---
// Instantiate the components for the reaction wheel system
Motor flywheel_motor(FLYWHEEL_PWM_PIN, FLYWHEEL_DIR_PIN, FLYWHEEL_BRAKE_PIN, FLYWHEEL_PWM_CHANNEL,
                     PWM_FREQ, PWM_RESOLUTION);
Sensor imu;
Encoder flywheel_encoder(FLYWHEEL_ENC_A_PIN, FLYWHEEL_ENC_B_PIN, ENCODER_PPR);
Estimator estimator(KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);
States robot_states(imu, flywheel_encoder, estimator);
Controller controller(K_GAINS);

// --- Timing Variables ---
unsigned long last_time;
const float Ts = 0.01f; // Control loop period: 10ms (100 Hz)

void setup()
{
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C for the IMU

    if (!imu.begin()) {
        Serial.println("IMU initialization failed! Halting.");
        while (1) {
            delay(100);
        }
    }

    flywheel_motor.releaseBrake();
    flywheel_encoder.reset();

    Serial.println("Reaction Wheel Pendulum Initialized. Starting balance control...");
    last_time = micros();
}

void loop()
{
    // 1. ---- TIMING CONTROL ----
    if (micros() - last_time < Ts * 1000000) {
        return;
    }
    float dt = (micros() - last_time) / 1000000.0f;
    last_time = micros();

    // 2. ---- UPDATE STATE ----
    robot_states.update(dt);

    // 3. ---- COMPUTE CONTROL SIGNAL ----
    float current_state[4] = {robot_states.phi, robot_states.phi_dot, robot_states.psi,
                              robot_states.psi_dot};
    float control_signal = controller.compute(current_state);

    // 4. ---- ACTUATE FLYWHEEL MOTOR ----
    flywheel_motor.setSpeed(control_signal);

    // Optional: Print states for debugging
    // Serial.printf("State: [%.2f, %.2f, %.2f, %.2f], Ctrl: %.2f\n",
    //               robot_states.phi, robot_states.phi_dot,
    //               robot_states.psi, robot_states.psi_dot, control_signal);
}