#ifdef MAIN_IDENTIFICATION
#include "main_identification.h"
#include "Motor.h"
#include <Arduino.h>

#define MOTOR_BRAKE_PIN 14
#define MOTOR_PWM_PIN 27
#define MOTOR_DIR_PIN 16
#define MOTOR_ENCA_PIN 25
#define MOTOR_ENCB_PIN 26

// Encoder specifics
#define ENCODER_TICKS_PER_REV 400 // 100 pulses per revolution, 400 ticks from quadrature

// Experiment Parameters
#define KD_SAMPLING_TIME_S 0.001f // 1 ms for transient experiment (Kd)
#define KW_SAMPLING_TIME_S 0.01f  // 10 ms for steady-state experiment (Kw)
#define STEADY_STATE_WAIT_MS 2000 // Time to wait for motor to reach steady state for Kw experiment
#define EXPERIMENT_DELAY_MS 500   // Short delay between PWM steps

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
    float deltaTicks = (float)(currentTicks - prevTicks);
    float revolutions = deltaTicks / ENCODER_TICKS_PER_REV;
    float radPerSec = (revolutions * 2.0f * PI) / deltaTimeSeconds;
    return radPerSec;
}

// Helper function to calculate angular acceleration in radians per second squared
// For Kd experiment, prevSpeedRadPerSec will be 0 as we measure from rest.
float calculateAccelerationRadPerSec2(float currentSpeedRadPerSec, float prevSpeedRadPerSec,
                                      float deltaTimeSeconds)
{
    if (deltaTimeSeconds == 0)
        return 0.0f;
    return (currentSpeedRadPerSec - prevSpeedRadPerSec) / deltaTimeSeconds;
}

void runKdExperiment()
{
    Serial.println("--- Start Kd Experiment (PWM, Acceleration) ---");
    long prevEncoderCount, currentEncoderCount;
    float deltaTime = KD_SAMPLING_TIME_S;

    for (int i = 0; i < NUM_KD_PWM_STEPS; ++i) {
        int pwmValue = KD_PWM_VALUES[i];
        Serial.print("START_DATA_KD_");
        Serial.print(pwmValue);
        Serial.println("_PWM");

        motor.brake(false); // Release brake
        motor.setPWM(0);    // Ensure motor is stopped
        delay(200);         // Settle time

        prevEncoderCount = motor.getEncoderCount();
        motor.setPWM(pwmValue);
        delayMicroseconds((unsigned long)(deltaTime * 10000.0f)); // Wait for sampling time
        currentEncoderCount = motor.getEncoderCount();
        motor.setPWM(0);   // Stop motor immediately after measurement
        motor.brake(true); // Apply brake

        float speedRadPerSec =
            calculateSpeedRadPerSec(currentEncoderCount, prevEncoderCount, deltaTime);
        // Initial acceleration assumes previous speed was 0
        float accelerationRadPerSec2 =
            calculateAccelerationRadPerSec2(speedRadPerSec, 0.0f, deltaTime);

        Serial.print(pwmValue);
        Serial.print(", ");
        Serial.println(accelerationRadPerSec2, 4); // Output with 4 decimal places

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
    float deltaTime = KW_SAMPLING_TIME_S;

    for (int i = 0; i < NUM_KW_PWM_STEPS; ++i) {
        int pwmValue = KW_PWM_VALUES[i];
        Serial.print("START_DATA_KW_");
        Serial.print(pwmValue);
        Serial.println("_PWM");

        motor.brake(false); // Release brake
        motor.setPWM(pwmValue);
        delay(STEADY_STATE_WAIT_MS); // Wait for motor to reach steady state

        prevEncoderCount = motor.getEncoderCount();
        delayMicroseconds((unsigned long)(deltaTime * 1000000.0f)); // Wait for sampling time
        currentEncoderCount = motor.getEncoderCount();

        motor.setPWM(0);   // Stop motor
        motor.brake(true); // Apply brake

        float speedRadPerSec =
            calculateSpeedRadPerSec(currentEncoderCount, prevEncoderCount, deltaTime);

        Serial.print(pwmValue);
        Serial.print(", ");
        Serial.println(speedRadPerSec, 4); // Output with 4 decimal places

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

    motor.begin();
    motor.brake(true); // Start with brake on

    Serial.println("Choose an experiment:");
    Serial.println("1: Kd Identification (Stall Torque - PWM vs Acceleration)");
    Serial.println("2: Kw Identification (Steady-State Speed - PWM vs Speed)");
    Serial.println("Enter choice (1 or 2):");
}

void loop()
{
    if (chosenExperiment == 0) {
        if (Serial.available() > 0) {
            char choice = Serial.read();
            if (choice == '1') {
                chosenExperiment = 1;
                runKdExperiment();
                // After experiment, remind user to choose again or reset
                Serial.println("\nExperiment finished. Choose again (1 or 2) or reset board:");
            } else if (choice == '2') {
                chosenExperiment = 2;
                runKwExperiment();
                Serial.println("\nExperiment finished. Choose again (1 or 2) or reset board:");
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
    // The loop will wait for a new choice if chosenExperiment is reset to 0
    // or do nothing if an experiment was just run and choice is still 1 or 2
    // until it's reset at the end of the experiment function.
    // A small delay can be good practice in loop if nothing is happening.
    delay(100);
}

#endif // MAIN_IDENTIFICATION