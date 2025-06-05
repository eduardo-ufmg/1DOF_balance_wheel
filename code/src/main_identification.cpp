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
#define KW_SAMPLING_TIME_S 0.01   // 10 ms for steady-state experiment (Kw) - use double
#define STEADY_STATE_WAIT_MS 2000 // Time to wait for motor to reach steady state for Kw experiment
#define EXPERIMENT_DELAY_MS 500   // Short delay between PWM steps

// Parameters for Kd experiment's high-frequency speed capture
#define NUM_KD_SPEED_SAMPLES 100          // Number of speed samples to capture
#define KD_SPEED_SAMPLING_INTERVAL_US 100 // Sampling interval in microseconds

// PWM Ranges (adjust as needed, Motor class takes -255 to 255)
// For Kd, we need small PWMs to measure initial acceleration from rest.
const int KD_PWM_VALUES[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50};
// const int KD_PWM_VALUES[] = {20, 30, 40, 50}; // Reduced set for faster testing
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
double calculateSpeedRadPerSec(long currentTicks, long prevTicks, double deltaTimeSeconds)
{
    if (deltaTimeSeconds == 0)
        return 0.0;
    double deltaTicks = static_cast<double>(currentTicks - prevTicks);
    double revolutions = deltaTicks / static_cast<double>(ENCODER_TICKS_PER_REV);
    double radPerSec = (revolutions * 2.0 * PI) / deltaTimeSeconds;
    return radPerSec;
}

void runKdExperiment()
{
    Serial.println("--- Start Kd Experiment (High-Frequency Speed Sampling) ---");
    Serial.println("Format: PWM, Timestamp (ms), Speed (rad/s)");

    std::vector<double> speedSamples(NUM_KD_SPEED_SAMPLES);
    std::vector<double> timeStampsMs(NUM_KD_SPEED_SAMPLES);

    double samplingIntervalSeconds = static_cast<double>(KD_SPEED_SAMPLING_INTERVAL_US) / 1000000.0;

    for (int i = 0; i < NUM_KD_PWM_STEPS; ++i) {
        int pwmValue = KD_PWM_VALUES[i];
        Serial.print("START_DATA_KD_");
        Serial.print(pwmValue);
        Serial.println("_PWM");

        Serial.print("Applied PWM: ");
        Serial.println(pwmValue);
        Serial.println("Timestamp (ms), Speed (rad/s)");

        motor.brake(false);      // Release brake
        motor.setPWM(0);         // Ensure motor is stopped and encoder count is stable
        motor.getEncoderCount(); // Clear encoder possibly
        delay(500);              // Settle time, ensure motor is truly at rest

        long prevEncoderCount = motor.getEncoderCount(); // Encoder count before PWM
        motor.setPWM(pwmValue);                          // Apply PWM

        unsigned long startTimeUs = micros(); // Reference time for this PWM step's sampling

        for (int k = 0; k < NUM_KD_SPEED_SAMPLES; ++k) {
            // Precise delay: busy wait until KD_SPEED_SAMPLING_INTERVAL_US has passed since last sample point
            // This is more accurate than repeated delayMicroseconds() calls if other code takes time.
            // For the first sample, (k+1)*KD_SPEED_SAMPLING_INTERVAL_US since startTimeUs
            // For subsequent samples, relative to the previous. Let's try absolute timing from start.
            while (micros() <
                   startTimeUs + (unsigned long)((k + 1) * KD_SPEED_SAMPLING_INTERVAL_US)) {
                // busy wait
            }
            long currentEncoderCount = motor.getEncoderCount();

            // Calculate time for this specific sample relative to the start of PWM application
            // More accurately, this is (k+1) * samplingIntervalSeconds
            double currentTimeSeconds = (k + 1) * samplingIntervalSeconds;
            timeStampsMs[k] = currentTimeSeconds * 1000.0; // Store time in ms

            speedSamples[k] = calculateSpeedRadPerSec(currentEncoderCount, prevEncoderCount,
                                                      samplingIntervalSeconds);
            prevEncoderCount = currentEncoderCount; // Update for the next interval
        }

        motor.setPWM(0);   // Stop motor immediately after all measurements for this step
        motor.brake(true); // Apply brake

        // Print all captured data for this PWM step
        for (int k = 0; k < NUM_KD_SPEED_SAMPLES; ++k) {
            Serial.print(timeStampsMs[k], 6); // Print time in ms with 6 decimal places
            Serial.print(", ");
            Serial.println(speedSamples[k], 6); // Print speed with 6 decimal places
        }

        Serial.print("END_DATA_KD_");
        Serial.print(pwmValue);
        Serial.println("_PWM");
        delay(EXPERIMENT_DELAY_MS); // Wait before next PWM step
    }
    Serial.println("--- End Kd Experiment ---");
    chosenExperiment = 0; // Reset for next choice
}

void runKwExperiment()
{
    Serial.println("--- Start Kw Experiment (PWM, Speed) ---");
    long prevEncoderCount, currentEncoderCount;
    double deltaTime = KW_SAMPLING_TIME_S; // Use double for precision

    for (int i = 0; i < NUM_KW_PWM_STEPS; ++i) {
        int pwmValue = KW_PWM_VALUES[i];
        Serial.print("START_DATA_KW_");
        Serial.print(pwmValue);
        Serial.println("_PWM");

        motor.brake(false); // Release brake
        motor.setPWM(pwmValue);
        delay(STEADY_STATE_WAIT_MS); // Wait for motor to reach steady state

        prevEncoderCount = motor.getEncoderCount();
        // Use delayMicroseconds for precise timing if deltaTime is small
        delayMicroseconds(static_cast<unsigned long>(deltaTime * 1000000.0));
        currentEncoderCount = motor.getEncoderCount();

        motor.setPWM(0);   // Stop motor
        motor.brake(true); // Apply brake

        double speedRadPerSec =
            calculateSpeedRadPerSec(currentEncoderCount, prevEncoderCount, deltaTime);

        Serial.print(pwmValue);
        Serial.print(", ");
        Serial.println(speedRadPerSec, 6); // Output with 6 decimal places

        Serial.print("END_DATA_KW_");
        Serial.print(pwmValue);
        Serial.println("_PWM");
        delay(EXPERIMENT_DELAY_MS); // Wait before next PWM step
    }
    Serial.println("--- End Kw Experiment ---");
    chosenExperiment = 0; // Reset for next choice
}

void setup()
{
    Serial.begin(115200);

    Serial.println("Motor Identification Program");
    Serial.println("Using double precision for calculations where applicable.");

    motor.begin();
    motor.brake(true); // Start with brake on

    Serial.println("Choose an experiment:");
    Serial.println("1: Kd Identification (High-Frequency Speed Sampling)");
    Serial.println("2: Kw Identification (Steady-State Speed - PWM vs Speed)");
    Serial.println("Enter choice (1 or 2):");
}

void loop()
{
    if (chosenExperiment == 0) {
        if (Serial.available() > 0) {
            char choice = Serial.read();
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
                    Serial.print("Invalid choice: ");
                    Serial.println(choice);
                }
                Serial.println("Please enter 1 or 2.");
            }
            // Clear any extra characters from the buffer
            while (Serial.available() > 0) {
                Serial.read();
            }
        }
    }
    // Small delay to prevent loop from running too fast if no serial input
    delay(100);
}

#endif // MAIN_IDENTIFICATION