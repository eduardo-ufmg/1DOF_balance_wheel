#pragma once
#include <Arduino.h>

namespace IMU {
    void init();
    void update();
    float getAngle();
}
