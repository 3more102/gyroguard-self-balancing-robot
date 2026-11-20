#include "watchdog_startup.h"

#include <avr/io.h>
#include <avr/wdt.h>

// .noinit preserves the reset cause captured by the .init3 startup hook.
uint8_t gyroguard_reset_cause __attribute__((section(".noinit")));

// ATmega328P keeps the watchdog active after a watchdog system reset.
// Disable it before application C-runtime initialization and setup() can time out again.
__attribute__((used, unused, naked, section(".init3")))
static void gyroguard_early_watchdog_init(void) {
  gyroguard_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
