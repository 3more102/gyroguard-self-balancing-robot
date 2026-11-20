#pragma once
#include <stdint.h>

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define INPUT 0

extern uint32_t fakeMillis;
extern uint16_t fakeAnalogValue;
inline uint32_t millis() { return fakeMillis; }
inline void pinMode(uint8_t, uint8_t) {}
inline int analogRead(uint8_t) { return static_cast<int>(fakeAnalogValue); }
