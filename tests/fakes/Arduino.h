#pragma once
#include <stdint.h>

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

extern uint32_t fakeMillis;
inline uint32_t millis() { return fakeMillis; }
