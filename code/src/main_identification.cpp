#ifdef MAIN_IDENTIFICATION
#include "main_identification.h"
#include "Motor.h"
#include <Arduino.h>
#include <vector>

#define MOTOR_BRAKE_PIN 14
#define MOTOR_PWM_PIN 27
#define MOTOR_DIR_PIN 16
#define MOTOR_ENCA_PIN 25
#define MOTOR_ENCB_PIN 26

// Encoder specifics
#define ENCODER_TICKS_PER_REV 400 // 100 pulses per revolution, 400 ticks from quadrature

// Experiment Parameters
#define KW_SAMPLING_TIME_S 0.01f  // 10 ms for steady-state experiment (Kw)
#define STEADY_STATE_WAIT_MS 5000 // Time to wait for motor to reach steady state for Kw experiment
#define EXPERIMENT_DELAY_MS 500   // Short delay between PWM steps

// Parameters for Kd experiment's angular displacement capture
#define KD_NUM_SAMPLES 100           // Number of displacement samples to capture per PWM step
#define KD_SAMPLING_INTERVAL_US 1000 // Sampling interval in microseconds (1ms)
#define KD_PWM_STEP_DURATION_MS                                                                    \
    (KD_NUM_SAMPLES * KD_SAMPLING_INTERVAL_US / 1000) // Duration for which each PWM is applied

// PWM Ranges (adjust as needed, Motor class takes -255 to 255)
// For Kd, we need small PWMs to measure initial acceleration from rest.
const int KD_PWM_VALUES[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50};
const int NUM_KD_PWM_STEPS = sizeof(KD_PWM_VALUES) / sizeof(KD_PWM_VALUES[0]);

// For Kw, we need a range of PWMs to get different steady-state speeds.
const int KW_PWM_VALUES[] = {5, 10, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200};
const int NUM_KW_PWM_STEPS = sizeof(KW_PWM_VALUES) / sizeof(KW_PWM_VALUES[0]);

// Global Motor Object
Motor motor(MOTOR_BRAKE_PIN, MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_ENCA_PIN, MOTOR_ENCB_PIN,
            ENCODER_TICKS_PER_REV);

// Experiment selection variable
int chosenExperiment = 0; // 0: None, 1: Kd, 2: Kw

// Helper function to calculate angular speed in radians per second
float calculateSpeedRadPerSec(long currentTicks, long prevTicks, float deltaTimeSeconds)
{
    if (deltaTimeSeconds == 0)
        return 0.0f;
    float deltaTicks = static_cast<float>(currentTicks - prevTicks);
    float revolutions = deltaTicks / static_cast<float>(ENCODER_TICKS_PER_REV);
    float radPerSec = (revolutions * 2.0f * PI) / deltaTimeSeconds;
    return radPerSec;
}

// Helper function to calculate absolute angular displacement in radians
float calculateAbsoluteAngleRad(long currentTicks, long initialTicks)
{
    float deltaTicks = static_cast<float>(currentTicks - initialTicks);
    float revolutions = deltaTicks / static_cast<float>(ENCODER_TICKS_PER_REV);
    return revolutions * 2.0f * PI;
}

void runKdExperiment()
{
    Serial.println("--- Start Kd Experiment (Angular Displacement Sampling) ---");
    Serial.println("Timestamp (ms), Absolute Angular Displacement (rad)");

    std::vector<float> displacementSamples(KD_NUM_SAMPLES);
    std::vector<float> timeStampsMs(KD_NUM_SAMPLES);

    for (int i = 0; i < NUM_KD_PWM_STEPS; ++i) {
        int pwmValue = KD_PWM_VALUES[i];
        Serial.print("PWM: ");
        Serial.println(pwmValue);

        motor.brake(false);        // Release brake
        motor.setPWM(0);           // Ensure motor is stopped
        motor.clearEncoderCount(); // Clear the count to start a new step
        delay(200); // Settle time, ensure motor is truly at rest and encoder count is stable

        long initialEncoderCount =
            motor.getEncoderCount(); // Encoder count at t=0 for this PWM step
        motor.setPWM(pwmValue);      // Apply PWM

        unsigned long stepStartTimeUs = micros(); // Reference time for this PWM step's sampling

        for (int k = 0; k < KD_NUM_SAMPLES; ++k) {
            // Precise delay until the next sampling point
            while (micros() <
                   stepStartTimeUs + (unsigned long)((k + 1) * KD_SAMPLING_INTERVAL_US)) {
                // busy wait
            }
            long currentEncoderCount = motor.getEncoderCount();
            unsigned long currentTimeUs = micros(); // Capture exact time of measurement

            // Calculate time for this specific sample relative to the start of PWM application
            timeStampsMs[k] =
                static_cast<float>(currentTimeUs - stepStartTimeUs) / 1000.0f; // Store time in ms

            displacementSamples[k] =
                calculateAbsoluteAngleRad(currentEncoderCount, initialEncoderCount);
        }

        motor.setPWM(0);   // Stop motor immediately after all measurements for this step
        motor.brake(true); // Apply brake

        // Print all captured data for this PWM step
        for (int k = 0; k < KD_NUM_SAMPLES; ++k) {
            Serial.print(timeStampsMs[k], 3); // Print time in ms with 3 decimal places
            Serial.print(", ");
            Serial.println(displacementSamples[k], 6); // Print displacement with 6 decimal places
        }

        delay(EXPERIMENT_DELAY_MS); // Wait before next PWM step
    }
    Serial.println("--- End Kd Experiment ---");
    chosenExperiment = 0; // Reset for next choice
}

void runKwExperiment()
{
    Serial.println("--- Start Kw Experiment (PWM, Speed) ---");
    Serial.println("PWM, Speed (rad/s)");
    long prevEncoderCount, currentEncoderCount;
    float deltaTime = KW_SAMPLING_TIME_S;

    for (int i = 0; i < NUM_KW_PWM_STEPS; ++i) {
        int pwmValue = KW_PWM_VALUES[i];

        motor.brake(false); // Release brake
        motor.setPWM(pwmValue);
        delay(STEADY_STATE_WAIT_MS); // Wait for motor to reach steady state

        prevEncoderCount = motor.getEncoderCount();
        // Use delayMicroseconds for precise timing if deltaTime is small
        delayMicroseconds(static_cast<unsigned long>(deltaTime * 1000000.0f));
        currentEncoderCount = motor.getEncoderCount();

        motor.setPWM(0);   // Stop motor
        motor.brake(true); // Apply brake

        float speedRadPerSec =
            calculateSpeedRadPerSec(currentEncoderCount, prevEncoderCount, deltaTime);

        Serial.print(pwmValue);
        Serial.print(", ");
        Serial.println(speedRadPerSec, 6); // Output with 6 decimal places

        delay(EXPERIMENT_DELAY_MS); // Wait before next PWM step
    }
    Serial.println("--- End Kw Experiment ---");
    chosenExperiment = 0; // Reset for next choice
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to be ready, especially for boards like ESP32-S3 etc.

    Serial.println("\nMotor Identification Program");

    motor.begin();
    motor.brake(true); // Start with brake on

    Serial.println("Choose an experiment:");
    Serial.println("1: Kd Identification (Angular Displacement Sampling)");
    Serial.println("2: Kw Identification (Steady-State Speed - PWM vs Speed)");
    Serial.println("Enter choice (1 or 2):");
}

void loop()
{
    if (chosenExperiment == 0) {
        if (Serial.available() > 0) {
            char choice = Serial.read();
            // Clear any extra characters from the buffer immediately after reading the choice
            while (Serial.available() > 0) {
                Serial.read();
            }

            if (choice == '1') {
                chosenExperiment = 1; // Mark experiment as chosen
                runKdExperiment();
                Serial.println("\nKd Experiment finished. Choose again (1 or 2) or reset board:");
            } else if (choice == '2') {
                chosenExperiment = 2; // Mark experiment as chosen
                runKwExperiment();
                Serial.println("\nKw Experiment finished. Choose again (1 or 2) or reset board:");
            } else {
                if (isprint(choice)) { // Avoid printing non-printable characters
                    Serial.print("Invalid choice: '");
                    Serial.print(choice);
                    Serial.println("'");
                }
                Serial.println("Please enter 1 or 2.");
            }
        }
    }
    // Small delay to prevent loop from running too fast if no serial input
    // and no experiment is running.
    if (chosenExperiment == 0) {
        delay(50);
    }
}

#endif // MAIN_IDENTIFICATION