#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

class MotorEncoder {
public:
  MotorEncoder(int pwmPin, int dirPin, int brakePin, int encAPin, int encBPin);
  void begin();
  void setPWM(float pwm); // -1.0 to 1.0
  void brake(bool on);
  long getPosition();
  void update();

private:
  int pwmPin, dirPin, brakePin, encAPin, encBPin;
  volatile long position;
  static void isrA();
  static void isrB();
  static MotorEncoder *instance;
};

#endif // MOTOR_ENCODER_H
