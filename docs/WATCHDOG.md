# AVR Watchdog Reset Recovery

GyroGuard targets the ATmega328P used by the Arduino Uno R3 and classic Nano.

## Startup handling

After a watchdog system reset, the AVR watchdog may remain enabled. GyroGuard disables it in the early .init3 startup section, before normal application initialization and C++ global constructors. The hook runs after the bootloader has transferred control to the sketch; it cannot protect a bootloader that itself loops on an active watchdog.

Use an Optiboot-based Uno or Nano new-bootloader configuration when relying on watchdog recovery. The old ATmegaBOOT Nano bootloader may reset-loop before the sketch starts.

## Reset-cause capture

- Optiboot may clear or modify MCUSR on some reset paths, but passes the original reset flags to the application in register r2. GyroGuard captures r2 in .init0 before normal AVR startup code can reuse that register.
- Legacy bootloaders may preserve MCUSR. The .init3 hook captures MCUSR and the application prefers it when nonzero.
- If MCUSR is zero, the reset policy uses the Optiboot handoff only when it contains the safety-relevant watchdog or brown-out flags.
- A watchdog flag combined with EXTRF is treated as an Optiboot upload-timeout event, not an application watchdog fault.
- An application watchdog reset (WDRF without EXTRF) or a brown-out reset causes a latched startup fault. The motor driver remains disabled; balance cannot be re-armed until the robot is inspected and power-cycled.

## Runtime watchdog

The sketch enables the runtime watchdog only after MPU-6050 identity validation and stationary calibration succeed. The main loop must continue calling wdt_reset() within the configured 250 ms timeout. The defensive wdt_disable() in setup() remains, but the .init3 hook is the primary startup recovery mechanism.

## Hardware safety

The watchdog hook recovers the MCU; it does not replace a hardware motor-disable path. Fit and verify the 10 kΩ pull-down on TB6612FNG STBY. Check the exact breakout for conflicting pull-ups. The firmware drives STBY low first when disabling the bridge.

## Validation

GitHub Actions tests the reset-cause policy on the host and compiles Uno plus both classic Nano bootloader targets. A physical watchdog-reset test must be performed with wheels raised, motor supply disconnected or current-limited, and STBY verified in standby before any powered balance test.
