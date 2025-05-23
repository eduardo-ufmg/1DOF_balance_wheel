// Main sketch for 1DOF Self-Balancing Robot
// Hardware: Wemos D1 R32, NIDEC24H, MPU6050
// Modular, extensible structure

#include "Config.h"
#include "IMU.h"
#include "Motor.h"
#include "Controller.h"

void setup() {
    Serial.begin(115200);
    IMU::init();
    Motor::init();
    Controller::init();
    Serial.println("1DOF Balance Wheel - Setup complete");
}

void loop() {
    IMU::update();
    Controller::update();
    Motor::update(Controller::getOutput());
    delay(5);
}
