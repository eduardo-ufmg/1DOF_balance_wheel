# 1DOF Inverted Pendulum with Reaction Wheel (ESP32, PlatformIO)

This project implements a self-balancing 1DOF inverted pendulum using a Wemos D1 R32 (ESP32) board, an MPU6050 IMU, and a NIDEC24H motor with encoder. The system uses a Kalman filter for tilt estimation, a state space controller for acceleration, and a PID controller for PWM output.

## Project Structure
- `code/` — Main firmware and source code
  - `src/` — Main source files (balancing and identification programs)
  - `include/` — Header files
  - `lib/` — Custom libraries:
    - `IMU/` — MPU6050 interface
    - `Motor/` — Motor driver and encoder
    - `KalmanFilter/` — Kalman filter for angle estimation
    - `StateSpaceController/` — State-space controller
    - `PIDController/` — PID controller for PWM
  - `platformio.ini` — PlatformIO configuration
- `identification/` — System identification data, scripts, and plots
- `modeling/` — System modeling notes and documentation

## Main Programs
- **control**: Main balancing program (`src/main.cpp`)
- **identification**: System identification program (`src/main_identification.cpp`)

## Board & Connections
- **Board**: Wemos D1 R32 (ESP32, PlatformIO env: `wemos_d1_uno32`)
- **IMU**: MPU6050 (I2C, addr 0x68, default ESP32 I2C pins)
- **Motor**: NIDEC24H
  - brake: io14
  - pwm: io27
  - dir: io16
  - enca: io25
  - encb: io26

## How to Build & Switch Programs
1. Install [PlatformIO](https://platformio.org/) (VSCode recommended).
2. Open the `code/` folder in PlatformIO/VSCode.
3. Use the `platformio.ini` environments to switch between `control` and `identification`:
   - Default: `control` (main balancing)
   - To build identification: select the `identification` environment in PlatformIO.
4. Upload to the board using PlatformIO's upload button or `pio run -t upload`.

## System Identification & Modeling
- Identification scripts and data are in `identification/` (Python scripts, measurement data, and plots for system parameters).
- Modeling notes and equations are in `modeling/` (Markdown files).

## Libraries & Dependencies
- All custom libraries are in `code/lib/`.
- External dependencies are managed by PlatformIO (see `platformio.ini`).

## License
MIT License — see [LICENSE](LICENSE)

---
This README will be updated as the project develops.
