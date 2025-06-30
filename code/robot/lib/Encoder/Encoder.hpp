#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <ESP32Encoder.h>

class Encoder
{
public:
    // Constructor initializes the encoder with the specified pins and pulses per revolution
    Encoder(int pinA, int pinB, int pulsesPerRevolution);

    // Reads the current count from the encoder
    int64_t getCount();

    // Reads the current angle from the encoder
    float getAngle();

    // Resets the encoder's count to zero
    void reset();

private:
    // The ESP32Encoder object
    ESP32Encoder _encoder;

    // The number of pulses per revolution for the encoder
    int _pulsesPerRevolution;
};

#endif // ENCODER_HPP