#pragma once

#include <stdint.h>

#ifndef A0
#define A0 14
#endif
#ifndef A1
#define A1 15
#endif
#ifndef A2
#define A2 16
#endif
#ifndef A3
#define A3 17
#endif
#ifndef INPUT
#define INPUT 0
#endif

extern uint16_t fakeAnalogReadValue;
inline void pinMode(uint8_t, uint8_t) {}
inline int analogRead(uint8_t) { return static_cast<int>(fakeAnalogReadValue); }
