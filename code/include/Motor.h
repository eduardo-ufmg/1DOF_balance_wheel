#pragma once

class Motor
{
public:
    Motor(int brakePin, int pwmPin, int dirPin, int encaPin, int encbPin);
    void begin();
    void setPWM(int pwm);
    void brake(bool on);
    long getEncoderCount();

private:
    int _brakePin, _pwmPin, _dirPin, _encaPin, _encbPin;
    volatile long _encoderCount;

    int _pwmChannel;
    int _pwmResolutionBits;
    int _pwmFrequency;

    // Static members to allow global ISR to access pin numbers
    static int s_encaPin;
    static int s_encbPin;

    // Friend declaration for the global ISR to access private members
    friend void encoderISR();
};

// Global instance pointer to allow ISR to access the motor object
extern Motor * g_motor_instance;