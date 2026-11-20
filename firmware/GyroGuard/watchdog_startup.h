#pragma once

#include <stdint.h>

// Captured from MCUSR before the AVR runtime clears or reinitializes state.
extern uint8_t gyroguard_reset_cause;
