#include "watchdog_startup.h"

#include <avr/io.h>
#include <avr/wdt.h>

volatile uint8_t gyroguard_mcusr_reset_cause __attribute__((section(".noinit")));
volatile uint8_t gyroguard_bootloader_reset_cause __attribute__((section(".noinit")));

// Optiboot clears MCUSR but passes the original reset flags to the application
// in r2. Capture r2 before the normal AVR startup code can reuse that register.
__attribute__((used, naked, section(".init0")))
static void gyroguard_capture_optiboot_reset_cause(void) {
  __asm__ __volatile__("sts %0, r2\n" : "=m" (gyroguard_bootloader_reset_cause));
}

// Legacy bootloaders may preserve MCUSR. The sketch selects MCUSR when nonzero
// and otherwise checks Optiboot's safety-relevant WDRF/BORF handoff.
__attribute__((used, unused, naked, section(".init3")))
static void gyroguard_early_watchdog_init(void) {
  gyroguard_mcusr_reset_cause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
