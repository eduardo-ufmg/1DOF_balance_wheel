#include "Config.hpp"
#include "KalmanFilter.hpp"
#include "MPU6050.hpp"
#include "Motor.hpp"
#include "PID.hpp"
#include "StateSpaceController.hpp"

// Global objects
MPU6050 imuHandler;
KalmanFilter angleKalmanFilter(KALMAN_Q_ANGLE, KALMAN_Q_BIAS, KALMAN_R_MEASURE);
Motor reactionWheelMotor(MOTOR_PIN_PWM, MOTOR_PIN_DIR, MOTOR_PIN_BRAKE,
                         MOTOR_PIN_ENCA, MOTOR_PIN_ENCB, MOTOR_PWM_CHANNEL,
                         MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION,
                         MOTOR_ENCODER_PPR);
PID motorAccelPID(MOTOR_PID_KP, MOTOR_PID_KI, MOTOR_PID_KD,
                  MOTOR_PID_OUTPUT_MIN, MOTOR_PID_OUTPUT_MAX);
StateSpaceController balanceController(STATE_SPACE_GAINS);

// Timing
unsigned long lastLoopTime_us = 0;
float dt_s = LOOP_PERIOD_MS / 1000.0f;  // Expected delta time in seconds

// State variables
float currentBodyAngle_rad = 0.0f;
float currentBodyAngularVelocity_rad_s = 0.0f;
// Wheel angular velocity is handled by motor.getSpeed_rad_s()
// Wheel actual acceleration is handled by motor.getAcceleration_rad_ss()

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);  // Wait for serial connection
  Serial.println("\n--- Inverted Pendulum Self-Balancer ---");

  // Initialize I2C
  Wire.begin();  // Uses default SDA (21), SCL (22) for ESP32

  // Initialize MPU6050
  if (!imuHandler.begin()) {
    Serial.println("MPU6050 initialization failed! Halting.");
    while (1) delay(100);
  }
  Serial.println("MPU6050 Initialized.");

  // Initialize Motor
  reactionWheelMotor.begin();
  reactionWheelMotor.releaseBrake();  // Ensure brake is off initially
  Serial.println("Motor Initialized.");

  // Initialize PID and State-Space Controllers
  motorAccelPID.reset();
  // State-Space controller initialized with gains from Config.hpp

  Serial.println("Controllers Initialized.");
  Serial.println("Setup complete. Starting balancing loop...");
  lastLoopTime_us = micros();
}

void loop() {
  unsigned long currentTime_us = micros();
  unsigned long elapsedTime_us = currentTime_us - lastLoopTime_us;

  if (elapsedTime_us >= (LOOP_PERIOD_MS * 1000)) {
    dt_s = elapsedTime_us / 1000000.0f;  // Actual delta time in seconds
    lastLoopTime_us = currentTime_us;

    // 1. Read IMU and Update Kalman Filter
    if (imuHandler.update()) {
      float accelAngle_rad = imuHandler.getAngleX_rad_from_accel();
      float gyroRateX_rad_s = imuHandler.getGyroX_rads();
      currentBodyAngle_rad =
          angleKalmanFilter.update(accelAngle_rad, gyroRateX_rad_s, dt_s);
      currentBodyAngularVelocity_rad_s =
          angleKalmanFilter.getRate();  // Bias-corrected rate
    } else {
      Serial.println("IMU update failed!");
      // Consider a safety stop if IMU fails repeatedly
      reactionWheelMotor.stop();
      return;
    }

    // 2. Update Motor State (reads encoder, calculates speed & acceleration)
    reactionWheelMotor.update(dt_s);
    float currentWheelAngularVelocity_rad_s =
        reactionWheelMotor.getSpeed_rad_s();
    float actualMotorAcceleration_rad_ss =
        reactionWheelMotor
            .getAcceleration_rad_ss();  // Estimated from speed changes

    // 3. Construct State Vector for Balance Controller
    float stateVector[STATE_DIMENSION] = {currentBodyAngle_rad,
                                          currentBodyAngularVelocity_rad_s,
                                          currentWheelAngularVelocity_rad_s};

    // 4. Compute Desired Motor Acceleration using State-Space Controller
    // Target: Bring pendulum to upright (angle = 0)
    // The state space gains should be tuned such that a positive angle (falling
    // forward) results in a motor acceleration that counteracts this fall.
    float desiredMotorAcceleration_rad_ss =
        balanceController.compute(stateVector);

    // 5. Compute Motor Effort using PID Controller
    // PID input: actual motor acceleration
    // PID setpoint: desired motor acceleration from state-space controller
    float motorEffort = motorAccelPID.compute(
        desiredMotorAcceleration_rad_ss, actualMotorAcceleration_rad_ss, dt_s);

    // 6. Actuate Motor
    reactionWheelMotor.setEffort(motorEffort);

    // --- Debugging Output (optional) ---
    // Keep this minimal in the final version to maintain loop speed
    // Serial.print("Time_ms: "); Serial.print(millis());
    // Serial.print(" dt_s: "); Serial.print(dt_s, 4);
    // Serial.print(" BodyAng(deg): "); Serial.print(currentBodyAngle_rad *
    // RAD_TO_DEG, 2); Serial.print(" BodyVel(dps): ");
    // Serial.print(currentBodyAngularVelocity_rad_s * RAD_TO_DEG, 2);
    // Serial.print(" WheelVel(rps): ");
    // Serial.print(currentWheelAngularVelocity_rad_s / (2*PI), 2);
    // Serial.print(" WheelAccelActual(rps^2): ");
    // Serial.print(actualMotorAcceleration_rad_ss / (2*PI), 2); Serial.print("
    // WheelAccelDesired(rps^2): ");
    // Serial.print(desiredMotorAcceleration_rad_ss / (2*PI), 2); Serial.print("
    // MotorEffort: "); Serial.print(motorEffort, 2); Serial.println();

    // Safety check (Example: if angle is too large, stop motor)
    if (abs(currentBodyAngle_rad * RAD_TO_DEG) >
        45.0f) {  // 45 degrees threshold
      Serial.println("Angle limit exceeded! Stopping motor.");
      reactionWheelMotor.stop();
      // Optional: Enter a safe state or require reset
    }
  }
}
