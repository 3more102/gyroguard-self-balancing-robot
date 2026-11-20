#pragma once
#include <stdint.h>

#define A0 14
#define A1 15
#define A3 17

extern uint32_t fakeMillis;
inline uint32_t millis() { return fakeMillis; }
