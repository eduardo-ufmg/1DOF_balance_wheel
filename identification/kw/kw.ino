/*
  ============================================
  UFMG BIA-Kit - K_omega Identification Experiment
  Copyright (c) 2024 by GTI (UFMG)
  ============================================

  This sketch is for Experiment 2: Steady-State Speed Characteristics.
  It measures the steady-state angular velocity of the wheel for different PWM duty cycles,
  to estimate the parameter K_omega in the empirical DC motor model.

  The robot body must be fixed (no tilt), and the wheel must start at rest for each trial.
  You must have already estimated K_D from the stall acceleration experiment.
*/

#include <Wire.h>
#include <ESP32Encoder.h>

// --- Pin definitions ---
#define BRAKE      14
#define NIDEC_PWM  27
#define DIR        16
#define ENCA       25
#define ENCB       26
#define NIDEC_PWM_CH 1
#define NIDEC_TIMER_BIT   8
#define NIDEC_BASE_FREQ   20000

// --- Experiment parameters ---
const float Ts = 0.01; // Sampling period [s] (10 ms)
const int N_AVG = 50;  // Number of samples to average for steady-state (~0.5s)
const int DUTY_CYCLES[] = {30, 60, 90, 120, 150, 180, 210, 240}; // PWM values (0-255)
const int N_DUTY = sizeof(DUTY_CYCLES)/sizeof(DUTY_CYCLES[0]);
const int N_TRIALS = 3; // Repeat each D for averaging

// --- Encoder and state ---
ESP32Encoder NIDEC_ENC;

// --- Utility functions ---
void MOTORcmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR, LOW);
  }
  ledcWrite(NIDEC_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

void NIDECsetup() {
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  pinMode(DIR, OUTPUT);
  ledcAttach(NIDEC_PWM, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  MOTORcmd(0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  NIDEC_ENC.attachFullQuad(ENCB, ENCA);
  NIDEC_ENC.clearCount();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  NIDECsetup();
  Serial.println("# K_omega Identification: D, trial, omega_ss [rad/s]");
  Serial.println("# Ensure the robot body is fixed and wheel starts at rest for each trial.");
}

void loop() {
  static bool done = false;
  if (done) return;

  for (int i = 0; i < N_DUTY; ++i) {
    int pwm = DUTY_CYCLES[i];
    for (int t = 0; t < N_TRIALS; ++t) {
      // 1. Ensure wheel is at rest
      MOTORcmd(0);
      NIDEC_ENC.clearCount();
      delay(1000);

      // 2. Apply PWM and wait for steady-state
      MOTORcmd(pwm);
      delay(2000); // Wait for wheel to reach steady-state

      // 3. Measure steady-state speed (average over N_AVG samples)
      float omega_sum = 0;
      long prev_count = NIDEC_ENC.getCount();
      for (int k = 0; k < N_AVG; ++k) {
        delay(int(Ts * 1000));
        long curr_count = NIDEC_ENC.getCount();
        float omega = (curr_count - prev_count) / 63.7f / Ts; // [rad/s], 63.7 counts/rev
        omega_sum += omega;
        prev_count = curr_count;
      }
      MOTORcmd(0);

      float omega_ss = omega_sum / N_AVG;

      Serial.print(pwm); Serial.print(", ");
      Serial.print(t+1); Serial.print(", ");
      Serial.println(omega_ss, 6);

      delay(1500); // Wait before next trial
    }
  }
  done = true;
  Serial.println("# Done. Copy data for K_omega estimation.");
}