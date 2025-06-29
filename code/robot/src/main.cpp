#include "Motor.hpp"
#include <Arduino.h>

// Define pins for the left and right motors
#define MOTOR_PWM_PIN 25
#define MOTOR_DIR_PIN 26
#define MOTOR_BRAKE_PIN 27

// Create a motor object
// Motor(pwmPin, dirPin, brakePin, pwmChannel)
Motor motor(MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_BRAKE_PIN, 0);

void setup()
{
    // Initialize the motor controller
    motor.begin();
}

void loop()
{
    // Example: Run motor forward at half speed for 2 seconds
    motor.setSpeed(0.5);
    delay(2000);

    // Stop the motor
    motor.setSpeed(0.0);
    delay(1000);

    // Run motor reverse at full speed for 2 seconds
    motor.setSpeed(-1.0);
    delay(2000);

    // Brake the motor
    motor.brake();
    delay(1000);

    // Release the brake
    motor.releaseBrake();
    delay(1000);
}