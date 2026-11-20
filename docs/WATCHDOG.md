# AVR Watchdog Reset Recovery

GyroGuard targets the ATmega328P used by the Arduino Uno R3 and classic Nano.

## Why early startup handling is required

On this AVR family, a watchdog system reset does not necessarily disable the watchdog. After a watchdog reset, it can remain active at its fastest prescaler. If firmware waits until Arduino `setup()` to disable it, the bootloader and runtime startup may consume the remaining watchdog interval and cause another reset before the application starts.

AVR-libc therefore recommends clearing the reset status and disabling the watchdog in the early `.init3` startup section.

## Implementation

`firmware/GyroGuard/watchdog_startup.cpp`:

1. Runs from `.init3`, before normal application initialization.
2. Copies `MCUSR` into the `.noinit` variable `gyroguard_reset_cause`.
3. Clears `MCUSR`.
4. Calls AVR-libc `wdt_disable()` using the required timed register sequence.

The sketch reports a previous watchdog reset over Serial. The normal application can then enable its runtime watchdog after initialization. The existing loop must continue calling `wdt_reset()` within the configured timeout.

The `wdt_disable()` call in `setup()` is retained as a defensive, harmless second disable; it is not the primary recovery mechanism.

## Hardware safety

The watchdog hook recovers the MCU; it does not replace a hardware motor-disable path. The TB6612FNG STBY input must remain low while the MCU is resetting. Check the exact motor-driver breakout for added pull-ups or other circuitry, and verify that the driver is in standby before connecting the motors.

## Validation

GitHub Actions compiles the sketch for the supported AVR board targets. A physical watchdog-reset test should be performed with the wheels raised, motor supply disconnected or current-limited, and the motor driver's standby state verified before any powered balance test.
