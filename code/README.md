# 1DOF Inverted Pendulum with Reaction Wheel (ESP32, PlatformIO)

This project implements a self-balancing 1DOF inverted pendulum using a Wemos D1 R32 (ESP32) board, an MPU6050 IMU, and a NIDEC24H motor with encoder. The system uses a Kalman filter for tilt estimation, a state space controller for acceleration, and a PID controller for PWM output.

## Structure
- `src/` — Main source files (control and identification programs)
- `include/` — Header files
- `lib/` — Custom libraries (MPU6050, Motor, Kalman, StateSpace, PID)

## Main Programs
- **control**: Main balancing program
- **identification**: For system identification

## Board & Connections
- **Board**: Wemos D1 R32 (wemos_d1_uno32)
- **IMU**: MPU6050 (I2C, addr 0x68, default ESP32 I2C pins)
- **Motor**: NIDEC24H
  - brake: io14
  - pwm: io27
  - dir: io16
  - enca: io25
  - encb: io26

## How to Build & Switch Programs
Use PlatformIO environments in `platformio.ini` to switch between `control` and `identification` mains.

---

This README will be updated as the project develops.
