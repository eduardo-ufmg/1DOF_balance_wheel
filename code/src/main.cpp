#ifdef MAIN_CONTROL
#include "IMU.h"
#include "LQR.h"
#include "Motor.h"
#include "main_control.h"
#include <Arduino.h>

// --- Pin Definitions ---
#define MOTOR_BRAKE_PIN 14
#define MOTOR_PWM_PIN 27
#define MOTOR_DIR_PIN 16
#define MOTOR_ENCA_PIN 25
#define MOTOR_ENCB_PIN 26
#define IMU_ADDRESS 0x68

// --- Encoder Specifics ---
#define ENCODER_TICKS_PER_REV 400

// --- Controller and timing ---
#define CONTROL_LOOP_TIME_MS 10 // 100Hz control loop
unsigned long lastLoopTime = 0;

// --- Object Declarations ---
IMU imu(IMU_ADDRESS);
Motor motor(MOTOR_BRAKE_PIN, MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_ENCA_PIN, MOTOR_ENCB_PIN,
            ENCODER_TICKS_PER_REV);
LQR lqr;

// --- Global variables for state ---
long lastEncoderCount = 0;
float wheel_speed = 0.0; // rad/s

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    // Initialize components
    if (!imu.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }

    motor.begin();
    motor.brake(false); // Release brake

    // Set the reference angle for the LQR controller (upright position)
    lqr.setReference(0.0);

    lastLoopTime = millis();
    lastEncoderCount = motor.getEncoderCount();
}

void loop()
{
    unsigned long currentTime = millis();
    if (currentTime - lastLoopTime >= CONTROL_LOOP_TIME_MS) {
        float dt = (currentTime - lastLoopTime) / 1000.0f; // Time delta in seconds
        lastLoopTime = currentTime;

        // --- 1. Update State Estimation ---
        imu.update();
        float angle = imu.getAngle();
        float rate = imu.getRate();

        long currentEncoderCount = motor.getEncoderCount();
        long deltaTicks = currentEncoderCount - lastEncoderCount;
        lastEncoderCount = currentEncoderCount;

        // Calculate wheel speed in rad/s
        wheel_speed = (float)deltaTicks / (float)ENCODER_TICKS_PER_REV * (2.0 * PI) / dt;

        // --- 2. Compute Control Signal ---
        float control_signal = lqr.compute(angle, rate, wheel_speed);

        // --- 3. Actuate Motor ---
        motor.setPWM((int)control_signal);
    }
}
#endif