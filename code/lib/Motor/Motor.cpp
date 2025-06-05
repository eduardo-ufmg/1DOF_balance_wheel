#include "Motor.h"
#include <Arduino.h>

// Global instance pointer definition
Motor * g_motor_instance = nullptr;

// Static member definitions
int Motor::s_encaPin = -1; // Initialize with invalid values
int Motor::s_encbPin = -1;

// Lock for critical section to safely access the encoder count
portMUX_TYPE _encoderMUX = portMUX_INITIALIZER_UNLOCKED;

// Existing global ISR from the provided file (empty initially)
// This ISR updates the encoder count of the g_motor_instance
void IRAM_ATTR encoderISR()
{
    if (g_motor_instance != nullptr) {
        // Read both encoder pins using the static pin numbers stored in the Motor class
        bool enca_state = digitalRead(g_motor_instance->s_encaPin);
        bool encb_state = digitalRead(g_motor_instance->s_encbPin);

        // Simple quadrature encoder logic (assumes interrupt on one edge of A)
        // If A is RISING (as configured in attachInterrupt in begin()), check B's state
        if (enca_state == HIGH) {
            if (encb_state == LOW) {
                g_motor_instance->_encoderCount++; // Clockwise
            } else {
                g_motor_instance->_encoderCount--; // Counter-clockwise
            }
        }
    }
}

Motor::Motor(int brakePin, int pwmPin, int dirPin, int encaPin, int encbPin)
    : _brakePin(brakePin), _pwmPin(pwmPin), _dirPin(dirPin), _encaPin(encaPin), _encbPin(encbPin),
      _encoderCount(0), _pwmChannel(0), _pwmResolutionBits(8),
      _pwmFrequency(10000) // Default values
{
    // Store encoder pin numbers in static members for access by the global ISR
    Motor::s_encaPin = encaPin;
    Motor::s_encbPin = encbPin;
}

void Motor::begin()
{
    // Set the global instance pointer to this motor object
    g_motor_instance = this;

    // Configure pin modes
    pinMode(_brakePin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_encaPin, INPUT_PULLUP); // Use INPUT_PULLUP for encoders
    pinMode(_encbPin, INPUT_PULLUP);

    // Initialize LEDC for PWM
    ledcSetup(_pwmChannel, _pwmFrequency, _pwmResolutionBits);
    ledcAttachPin(_pwmPin, _pwmChannel);
    ledcWrite(_pwmChannel, 0); // Start with motor off

    // Set brake off initially (active low, so HIGH means off)
    digitalWrite(_brakePin, HIGH);

    // Attach interrupt for encoder pin A
    // Interrupt will trigger on the RISING edge of _encaPin, and encoderISR will update the count
    attachInterrupt(digitalPinToInterrupt(_encaPin), encoderISR, RISING);
}

void Motor::setPWM(int pwm)
{
    // Ensure PWM value is within the valid range for an 8-bit resolution (-255 to 255)
    pwm = constrain(pwm, -255, 255);

    // Determine direction
    if (pwm > 0) {
        digitalWrite(_dirPin, HIGH); // Set direction for positive PWM
    } else if (pwm < 0) {
        digitalWrite(_dirPin, LOW); // Set direction for negative PWM
    } else {
        // keep direction unchanged if pwm is 0
    }

    // Get absolute PWM value to set duty cycle
    int dutyCycle = abs(pwm);

    // Calculate maximum duty cycle for the given resolution
    int maxDutyCycle = (1 << _pwmResolutionBits) - 1; // e.g., 2^8 - 1 = 255 for 8-bit

    // PWM is active low/inverted polarity, so subtract from max duty cycle
    int invertedDutyCycle = maxDutyCycle - dutyCycle;

    // Write the inverted duty cycle to the LEDC channel
    ledcWrite(_pwmChannel, invertedDutyCycle);
}

void Motor::brake(bool on)
{
    // Brake is active low: LOW means brake ON, HIGH means brake OFF
    digitalWrite(_brakePin, on ? LOW : HIGH);
}

long Motor::getEncoderCount()
{
    // Use a critical section to safely read the encoder count, as it's modified by an ISR
    long count;
    taskENTER_CRITICAL(&_encoderMUX); // Enter critical section
    count = _encoderCount;
    taskEXIT_CRITICAL(&_encoderMUX); // Exit critical section
    return count;
}