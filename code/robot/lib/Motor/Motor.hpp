#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>

/**
 * @class Motor
 * @brief Controls a Nidec 24H404H070 brushless DC motor.
 *
 * This class provides a high-level interface for controlling the motor's speed
 * and direction using PWM. It also handles the motor's braking mechanism.
 * The interface is designed to be transparent, abstracting away the hardware-specific
 * details like active-low signals.
 */
class Motor
{
public:
    /**
     * @brief Construct a new Motor object.
     *
     * @param pwmPin The pin for the PWM speed control signal.
     * @param dirPin The pin for the motor direction control.
     * @param brakePin The pin for the motor brake control (active low).
     * @param pwmChannel The ESP32 LEDC PWM channel to use (0-15).
     * @param pwmFrequency The PWM frequency in Hz. Defaults to 20 kHz.
     * @param pwmResolution The PWM resolution in bits. Defaults to 8 bits.
     */
    Motor(uint8_t pwmPin, uint8_t dirPin, uint8_t brakePin, uint8_t pwmChannel = 0,
          uint32_t pwmFrequency = 20000, uint8_t pwmResolution = 8);

    /**
     * @brief Initializes the motor controller.
     *
     * Sets up the PWM channel and pin modes. This should be called in the
     * setup() function of the main sketch.
     */
    void begin();

    /**
     * @brief Sets the speed and direction of the motor.
     *
     * @param speed A value from -1.0 (full speed reverse) to 1.0 (full speed forward).
     * A value of 0.0 will stop the motor.
     */
    void setSpeed(float speed);

    /**
     * @brief Activates the motor's brake.
     */
    void brake();

    /**
     * @brief Releases the motor's brake.
     */
    void releaseBrake();

private:
    // Pin assignments
    uint8_t _pwmPin;
    uint8_t _dirPin;
    uint8_t _brakePin;

    // PWM configuration
    uint8_t _pwmChannel;
    uint32_t _pwmFrequency;
    uint8_t _pwmResolution;
    uint32_t _maxDutyCycle;
};

#endif // MOTOR_HPP