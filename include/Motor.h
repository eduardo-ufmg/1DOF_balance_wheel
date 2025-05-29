#pragma once
#include <Arduino.h>

namespace Motor {
    void init();
    void update(float controlSignal);
}
