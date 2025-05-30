/*
  ============================================
  UFMG BIA-Kit - KD Identification Experiment
  Copyright (c) 2024 by GTI (UFMG)
  ============================================

  This sketch is for Experiment 1: Stall Torque Characteristics.
  It measures the initial angular acceleration of the wheel for different PWM duty cycles,
  to estimate the parameter K_D in the empirical DC motor model.

  The robot body must be fixed (no tilt), and the wheel must start at rest for each trial.
*/

#include <Wire.h>
#include <ESP32Encoder.h>

// --- Pin definitions (adapt as needed) ---
#define BRAKE      14
#define NIDEC_PWM  27
#define DIR        16
#define ENCA       25
#define ENCB       26
#define NIDEC_PWM_CH 1
#define NIDEC_TIMER_BIT   8
#define NIDEC_BASE_FREQ   20000

// --- Experiment parameters ---
const float Ts = 0.001; // Sampling period [s] (1 ms)
const int N_SAMPLES = 20; // Number of samples for initial acceleration fit (~20 ms)
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
  Serial.println("# KD Identification: D, trial, accel_init [rad/s^2]");
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

      // 2. Apply PWM and record encoder counts
      unsigned long t0 = micros();
      float time[N_SAMPLES], pos[N_SAMPLES];
      for (int k = 0; k < N_SAMPLES; ++k) {
        if (k == 0) MOTORcmd(pwm);
        time[k] = (micros() - t0) / 1e6; // seconds
        pos[k] = NIDEC_ENC.getCount() / 63.7f; // [rad], 63.7 counts/rev, 2pi/63.7 rad/count
        delayMicroseconds(int(Ts * 1e6));
      }
      MOTORcmd(0);

      // 3. Estimate initial acceleration (fit a line to first ~20 ms of speed)
      // Use linear regression on (time, pos) to get velocity, then diff for acceleration
      float v[N_SAMPLES-1], t_v[N_SAMPLES-1];
      for (int k = 0; k < N_SAMPLES-1; ++k) {
        v[k] = (pos[k+1] - pos[k]) / (time[k+1] - time[k]);
        t_v[k] = 0.5f * (time[k+1] + time[k]);
      }
      // Fit a line v = a*t + b to (t_v, v)
      float sum_t = 0, sum_v = 0, sum_tt = 0, sum_tv = 0;
      for (int k = 0; k < N_SAMPLES-1; ++k) {
        sum_t += t_v[k];
        sum_v += v[k];
        sum_tt += t_v[k]*t_v[k];
        sum_tv += t_v[k]*v[k];
      }
      float denom = (N_SAMPLES-1)*sum_tt - sum_t*sum_t;
      float accel_init = 0;
      if (denom != 0) {
        accel_init = ((N_SAMPLES-1)*sum_tv - sum_t*sum_v) / denom;
      }

      Serial.print(pwm); Serial.print(", ");
      Serial.print(t+1); Serial.print(", ");
      Serial.println(accel_init, 6);

      delay(1500); // Wait before next trial
    }
  }
  done = true;
  Serial.println("# Done. Copy data for KD estimation.");
}