#pragma once

#include <stdint.h>

extern uint32_t fakeMillis;
inline uint32_t millis() { return fakeMillis; }

// Arduino Uno analog pins are also exposed as digital pin numbers.
constexpr uint8_t A0 = 14;
constexpr uint8_t A1 = 15;
constexpr uint8_t A3 = 17;
