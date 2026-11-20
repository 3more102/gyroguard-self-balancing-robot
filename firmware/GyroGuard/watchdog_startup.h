#pragma once

#include <stdint.h>

// Captured from MCUSR before the AVR runtime clears or reinitializes state.
extern volatile uint8_t gyroguard_reset_cause;

// A validated .noinit marker records that balance was armed before a reset.
bool gyroguard_take_previous_armed_marker();
void gyroguard_mark_balance_armed();
void gyroguard_clear_armed_marker();
