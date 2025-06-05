#include "KalmanFilter.h"
#include "MPU6050Interface.h"
#include "MotorEncoder.h"
#include "PIDController.h"
#include "StateSpaceController.h"
#include <Arduino.h>

// Pin definitions
#define BRAKE_PIN 14
#define PWM_PIN 27
#define DIR_PIN 16
#define ENCA_PIN 25
#define ENCB_PIN 26

MPU6050Interface imu;
KalmanFilter kalman;
StateSpaceController ssController;
PIDController pid(1.0, 0.0, 0.0);
MotorEncoder motor(PWM_PIN, DIR_PIN, BRAKE_PIN, ENCA_PIN, ENCB_PIN);

void setup() {
  Serial.begin(115200);
  imu.begin();
  kalman.initialize(0.001, 0.003, 0.03);
  motor.begin();
  // Set up state-space controller parameters here
}

void loop() {
  float accel, gyro;
  if (imu.read(accel, gyro)) {
    float dt = 0.01; // 10 ms loop
    float angle = kalman.update(accel, gyro, dt);
    float state[2] = {angle, gyro};
    float acc_cmd = ssController.compute(state, 0.0f);
    float pwm = pid.compute(acc_cmd, 0.0f, dt);
    motor.setPWM(pwm);
  }
  delay(10);
}
