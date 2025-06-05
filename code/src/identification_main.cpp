#include "MPU6050Interface.h"
#include "MotorEncoder.h"
#include <Arduino.h>

#define BRAKE_PIN 14
#define PWM_PIN 27
#define DIR_PIN 16
#define ENCA_PIN 25
#define ENCB_PIN 26

MPU6050Interface imu;
MotorEncoder motor(PWM_PIN, DIR_PIN, BRAKE_PIN, ENCA_PIN, ENCB_PIN);

void setup() {
  Serial.begin(115200);
  imu.begin();
  motor.begin();
}

void loop() {
  float accel, gyro;
  if (imu.read(accel, gyro)) {
    Serial.print("Accel: ");
    Serial.print(accel);
    Serial.print(", Gyro: ");
    Serial.println(gyro);
  }
  Serial.print("Encoder: ");
  Serial.println(motor.getPosition());
  delay(50);
}
