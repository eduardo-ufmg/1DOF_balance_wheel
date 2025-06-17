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

// Global Motor Object
Motor motor(MOTOR_BRAKE_PIN, MOTOR_PWM_PIN, MOTOR_DIR_PIN, MOTOR_ENCA_PIN, MOTOR_ENCB_PIN,
            ENCODER_TICKS_PER_REV);

void setup()
{
}

void loop()
{
}

#endif // MAIN_IDENTIFICATION