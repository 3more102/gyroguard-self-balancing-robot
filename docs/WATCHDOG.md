# AVR Watchdog Reset Recovery

GyroGuard targets the ATmega328P used by the Arduino Uno R3 and classic Nano.

## Why early startup handling is required

After a watchdog system reset, the watchdog may remain enabled at its fastest prescaler. If the sketch waits until Arduino `setup()` to disable it, the application's C-runtime startup can consume the remaining watchdog interval and trigger another reset before `setup()` runs.

The `.init3` hook runs after the bootloader has transferred control to the sketch, but before normal application initialization and C++ global constructors. It disables the watchdog early enough to protect the sketch's startup sequence.

**Important bootloader boundary:** `.init3` does not execute before or during bootloader code. If the watchdog can time out while the bootloader is running, that bootloader must itself disable or safely manage the watchdog. Verify the exact bootloader installed on the Uno/Nano being used; bootloader variants are not interchangeable for this behavior.

## Implementation

`firmware/GyroGuard/watchdog_startup.cpp`:

1. Runs from `.init3`, before normal application initialization.
2. Copies `MCUSR` into the `.noinit` variable `gyroguard_reset_cause`.
3. Clears `MCUSR`.
4. Calls AVR-libc `wdt_disable()` using the required timed register sequence.

The sketch reports a previous watchdog reset over Serial. The normal application can then enable its runtime watchdog after initialization. The existing loop must continue calling `wdt_reset()` within the configured timeout.

The `wdt_disable()` call in `setup()` is retained as a defensive, harmless second disable; it is not the primary recovery mechanism.

## Hardware safety

The watchdog hook recovers the MCU; it does not replace a hardware motor-disable path. The TB6612FNG STBY input must remain low while the MCU is resetting. Check the exact motor-driver breakout for added pull-ups or other circuitry. If the module has no verified STBY pull-down, add an external 10 kΩ resistor from STBY to GND; do not add it blindly if the breakout already pulls STBY high. Verify the driver remains in standby during reset before connecting the motors.

## Validation

GitHub Actions compiles the sketch for the supported AVR board targets. A physical watchdog-reset test should be performed with the wheels raised, motor supply disconnected or current-limited, and the motor driver's standby state verified before any powered balance test.
