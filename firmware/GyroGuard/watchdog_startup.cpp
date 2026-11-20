#include "watchdog_startup.h"
#include "reset_policy.h"

#include <avr/io.h>
#include <avr/wdt.h>

// .noinit preserves reset state across MCU resets; it is not C-runtime initialized.
volatile uint8_t gyroguard_reset_cause __attribute__((section(".noinit")));
volatile uint16_t gyroguard_armed_marker_magic __attribute__((section(".noinit")));
volatile uint16_t gyroguard_armed_marker_inverse __attribute__((section(".noinit")));

bool gyroguard_take_previous_armed_marker() {
  const uint16_t magic = gyroguard_armed_marker_magic;
  const uint16_t inverse = gyroguard_armed_marker_inverse;
  gyroguard_clear_armed_marker();
  return GyroGuardReset::isValidArmedMarker(magic, inverse);
}

void gyroguard_mark_balance_armed() {
  // Write the inverse first and the magic last; the marker is valid only once
  // both words have been committed.
  gyroguard_armed_marker_inverse = GyroGuardReset::kArmedMarkerInverse;
  gyroguard_armed_marker_magic = GyroGuardReset::kArmedMarkerMagic;
}

void gyroguard_clear_armed_marker() {
  // Invalidate first so an interruption cannot leave a partially valid marker.
  gyroguard_armed_marker_magic = 0;
  gyroguard_armed_marker_inverse = 0;
}

// ATmega328P keeps the watchdog active after a watchdog system reset.
// Disable it before application C-runtime initialization and setup() can time out again.
__attribute__((used, unused, naked, section(".init3")))
static void gyroguard_early_watchdog_init(void) {
  gyroguard_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
