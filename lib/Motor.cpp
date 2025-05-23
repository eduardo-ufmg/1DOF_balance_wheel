#include "Motor.h"
#include "Config.h"

namespace Motor {
    void init() {
        pinMode(MOTOR_PWM_PIN, OUTPUT);
        pinMode(MOTOR_DIR_PIN, OUTPUT);
    }
    void update(float controlSignal) {
        // Control NIDEC24H motor
    }
}
