#include "Motor.h"
#include <Arduino.h> // Required for Arduino functions like pinMode, digitalWrite, etc.

Motor::Motor(int brakePin, int pwmPin, int dirPin, int encaPin, int encbPin, int ticksPerRev)
    : _brakePin(brakePin), _pwmPin(pwmPin), _dirPin(dirPin), _encaPin(encaPin), _encbPin(encbPin),
      _ticksPerRev(ticksPerRev), _pwmChannel(0), _pwmResolutionBits(8),
      _pwmFrequency(10000), // Default values
      _encoder()            // Initialize ESP32Encoder object
{
    // The encoder pins are stored in _encaPin and _encbPin,
    // and will be attached in the begin() method.
}

void Motor::begin()
{
    // Configure pin modes for motor control
    pinMode(_brakePin, OUTPUT);
    pinMode(_dirPin, OUTPUT);

    // Initialize LEDC for PWM
    ledcSetup(_pwmChannel, _pwmFrequency, _pwmResolutionBits);
    ledcAttachPin(_pwmPin, _pwmChannel);
    ledcWrite(_pwmChannel, 0); // Start with motor off

    // Set brake off initially (active low, so HIGH means off)
    digitalWrite(_brakePin, HIGH);

    // Initialize the encoder using ESP32Encoder library
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    _encoder.attachFullQuad(_encaPin, _encbPin); // Attach encoder to pins
    _encoder.setCount(0);                        // Reset encoder count to 0
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
    // Simply return the count from the ESP32Encoder object
    return _encoder.getCount();
}

int Motor::getTicksPerRev()
{
    // Return the ticks per revolution set during initialization
    return _ticksPerRev;
}
