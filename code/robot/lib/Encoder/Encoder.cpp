#include "Encoder.hpp"
#include <math.h>

Encoder::Encoder(int pinA, int pinB, int pulsesPerRevolution)
{
    // Stores the number of pulses per revolution for the encoder
    _pulsesPerRevolution = pulsesPerRevolution;

    // The ESP32Encoder library handles all the setup internally.
    // We just need to attach the GPIO pins.
    _encoder.attachFullQuad(pinA, pinB);

    // Clear the encoder's count on startup
    _encoder.clearCount();
}

int64_t Encoder::getCount()
{
    return _encoder.getCount();
}

/**
 * @brief Calculates and returns the current angle of the encoder in radians.
 * 
 * This function computes the angle based on the encoder's count and the 
 * number of pulses per revolution. The angle is calculated using the formula:
 * 
 *     angle = (count * 2 * π) / pulsesPerRevolution
 * 
 * @return float The current angle of the encoder in radians.
 */
float Encoder::getAngle()
{
    // Calculate the angle based on the count and pulses per revolution
    return static_cast<float>(_encoder.getCount()) * 2.0f * M_PI /
           static_cast<float>(_pulsesPerRevolution);
}

void Encoder::reset()
{
    _encoder.clearCount();
}