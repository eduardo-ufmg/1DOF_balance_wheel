#include "MotorEncoder.h"

MotorEncoder *MotorEncoder::instance = nullptr;

MotorEncoder::MotorEncoder(int pwmPin, int dirPin, int brakePin, int encAPin,
                           int encBPin)
    : pwmPin(pwmPin), dirPin(dirPin), brakePin(brakePin), encAPin(encAPin),
      encBPin(encBPin), position(0) {
  instance = this;
}

void MotorEncoder::begin() {
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  pinMode(encAPin, INPUT_PULLUP);
  pinMode(encBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encAPin), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBPin), isrB, CHANGE);
}

void MotorEncoder::setPWM(float pwm) {
  bool dir = pwm >= 0;
  analogWrite(pwmPin, (int)(fabs(pwm) * 255));
  digitalWrite(dirPin, dir);
}

void MotorEncoder::brake(bool on) { digitalWrite(brakePin, on ? HIGH : LOW); }

long MotorEncoder::getPosition() { return position; }

void MotorEncoder::update() {
  // Optionally update position or handle encoder logic
}

void MotorEncoder::isrA() {
  if (instance) {
    int a = digitalRead(instance->encAPin);
    int b = digitalRead(instance->encBPin);
    if (a == b)
      instance->position++;
    else
      instance->position--;
  }
}

void MotorEncoder::isrB() {
  if (instance) {
    int a = digitalRead(instance->encAPin);
    int b = digitalRead(instance->encBPin);
    if (a != b)
      instance->position++;
    else
      instance->position--;
  }
}
