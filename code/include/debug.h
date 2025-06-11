#pragma once
#include <Arduino.h>

// --- General Debugging ---
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)

// --- Timed Debugging Control ---
// Interval in milliseconds
#define DEBUG_INTERVAL_MS 500
static unsigned long lastDebugTime = 0;

// Macro to start a timed debug block. It checks if the interval has passed.
#define START_DEBUG_CYCLE()                                                                        \
    if (millis() - lastDebugTime > DEBUG_INTERVAL_MS) {                                            \
        lastDebugTime = millis();

// Macro to end a timed debug block
#define END_DEBUG_CYCLE() }

#else
// If DEBUG is off, all debug-related macros are empty and will be compiled out.
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define START_DEBUG_CYCLE() if (false) {
#define END_DEBUG_CYCLE() }
#endif