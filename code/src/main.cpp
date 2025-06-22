#ifdef MAIN_CONTROL
#include "Controller.h"
#include "IMU.h"
#include "Motor.h"
#include "debug.h"
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

// --- Motor PWM Settings ---
#define MOTOR_PWM_FREQ 10000
#define MOTOR_PWM_RESOLUTION 8 // Bits

// --- CONTROL SCALE ---
#define CONTROL_SCALE (float)((1 << MOTOR_PWM_RESOLUTION) - 1)

// --- Controller and timing ---
#define CONTROL_LOOP_TIME_MS 10 // 100Hz control loop
unsigned long lastLoopTime = 0;

// --- Object Declarations ---
IMU imu(IMU_ADDRESS);
Motor motor(MOTOR_BRAKE_PIN, MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_ENCA_PIN, MOTOR_ENCB_PIN,
            ENCODER_TICKS_PER_REV, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
Controller controller;

// --- Global variables for state ---
long lastEncoderCount = 0;
float wheel_position = 0.0; // rad
float wheel_speed = 0.0;    // rad/s

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    DEBUG_PRINTLN("Serial monitor initialized.");

    // Initialize components
    if (!imu.begin()) {
        DEBUG_PRINTLN("Failed to initialize IMU!");
        while (1)
            ;
    }

    // --- Initialization Procedure ---
    delay(5000); // Give user time to position the robot

    imu.calibrate(); // Calibrate IMU and pre-converge filter
    DEBUG_PRINTLN("IMU calibrated.");

    // Set the reference angle for the controller (upright position)
    controller.setReference(0.0);
    DEBUG_PRINTLN("Controller reference set to 0.0.");

    motor.begin();
    motor.brake(false); // Release brake
    DEBUG_PRINTLN("Motor initialized and brake released.");

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
        float angle = -imu.getAngle();
        float rate = -imu.getRate();

        long currentEncoderCount = motor.getEncoderCount();
        long deltaTicks = currentEncoderCount - lastEncoderCount;
        lastEncoderCount = currentEncoderCount;

        // Calculate wheel position in radians
        wheel_position = (float)currentEncoderCount / (float)ENCODER_TICKS_PER_REV * (2.0 * PI);

        // Calculate wheel speed in rad/s
        wheel_speed = (float)deltaTicks / (float)ENCODER_TICKS_PER_REV * (2.0 * PI) / dt;

        // --- 2. Compute Control Signal ---
        float control_signal =
            controller.compute(angle, rate, wheel_position, wheel_speed) * CONTROL_SCALE;

        // --- 3. Actuate Motor ---
        motor.setPWM((int)control_signal);

        // --- 4. Timed Debug Output ---
        START_DEBUG_CYCLE()
        DEBUG_PRINT("Angle (°): ");
        DEBUG_PRINT(angle * 180.0 / PI);
        DEBUG_PRINT(",\tControl Signal (duty): ");
        DEBUG_PRINT(control_signal);
        DEBUG_PRINT(",\tWheel Speed (rad/s): ");
        DEBUG_PRINT(wheel_speed);
        DEBUG_PRINT(",\tWheel Pos (rad): ");
        DEBUG_PRINTLN(wheel_position);
        END_DEBUG_CYCLE()
    }
}
#endif