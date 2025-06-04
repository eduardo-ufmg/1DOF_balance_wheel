#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>
#include <ESP32Encoder.h> // Requires ESP32Encoder library

class Motor {
public:
  Motor(int pwmPin, int dirPin, int brakePin, int encAPin, int encBPin,
        int pwmChannel, int pwmFrequency, int pwmResolution, float encoderPPR);

  void begin();
  void update(float dt); // Call periodically to update speed and acceleration

  // Set motor effort: -1.0 (full reverse) to 1.0 (full forward)
  void setEffort(float effort);
  void activateBrake();
  void releaseBrake();
  void stop(); // Stops motor and activates brake

  float getSpeed_rad_s() const; // Angular speed in radians per second
  float
  getAcceleration_rad_ss() const; // Angular acceleration in rad/s^2 (estimated)
  long getEncoderCounts() const;
  float getDisplacement_rad() const; // Total angular displacement in radians

private:
  // Pins
  int _pwmPin, _dirPin, _brakePin;
  int _encAPin, _encBPin;

  // PWM
  int _pwmChannel, _pwmFrequency, _pwmResolution;
  int _maxDutyCycle;

  // Encoder
  ESP32Encoder _encoder;
  float _encoderPPR;
  float _radsPerCount;
  long _lastEncoderCount;

  // State
  float _currentSpeed_rad_s;
  float _previousSpeed_rad_s;
  float _currentAcceleration_rad_ss;
  bool _brakeActive;
};

#endif // MOTOR_HPP
