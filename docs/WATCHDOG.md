# AVR Watchdog Reset Recovery

GyroGuard targets the ATmega328P used by the Arduino Uno R3 and classic Nano.

## Early watchdog shutdown

After a watchdog system reset, the watchdog may remain enabled at its fastest prescaler. If the sketch waits until Arduino `setup()` to disable it, the application's C-runtime startup can consume the remaining watchdog interval and trigger another reset before `setup()` runs.

The `.init3` hook in `firmware/GyroGuard/watchdog_startup.cpp` runs after the bootloader transfers control to the sketch but before normal C/C++ initialization. It captures `MCUSR`, clears the register and disables the watchdog before the application starts.

This hook cannot run before or during bootloader execution. Verify the exact bootloader installed on the Uno/Nano being used.

## Armed-state reset marker

Reset-cause flags alone are not portable across all Arduino bootloaders. The official Arduino AVR Optiboot source reads and clears `MCUSR` before jumping to the sketch; therefore the application may receive zero reset flags on stock Optiboot. GyroGuard does **not** depend on an undocumented `r2` register handoff. See the [official Arduino AVR Optiboot source](https://github.com/arduino/ArduinoCore-avr/blob/master/bootloaders/optiboot/optiboot.c).

Instead, GyroGuard uses a two-word marker in AVR `.noinit` SRAM:

- The marker is written immediately before `balanceArmed` becomes true.
- A magic value and its bitwise inverse must both match; this reduces false positives from arbitrary SRAM contents.
- The marker is cleared after deliberate `d` disarm and after a latched fault has disabled the motor driver.
- At startup, the firmware reads and clears the marker. If it was valid, the firmware latches a startup fault and returns before enabling the balance controller or runtime watchdog.
- `MCUSR` watchdog and brown-out flags remain supplemental detection when the installed bootloader preserves them.

This deliberately treats **any reset while balance was armed** as unsafe, even if the exact reset source cannot be identified. It does not automatically re-arm the robot. The operator must inspect the chassis, battery, motor driver and IMU before power-cycling and deliberately sending `b` again.

The marker relies on SRAM retention through the installed bootloader's reset path. A power loss or severe brown-out may destroy SRAM contents; retain the hardware STBY pull-down, protected battery pack and physical disconnect. This software marker is not a substitute for those protections.

## Motor-driver shutdown

The TB6612FNG STBY input must remain low while the MCU is resetting. Use a verified external 10 kΩ pull-down from STBY to GND unless the exact breakout already has a suitable pull-down. `MotorDriver::disable()` drives STBY low before changing PWM or direction outputs.

## Bench-only reset-marker test

The firmware contains a test-only path guarded by `GYROGUARD_TEST_WATCHDOG_RESET`. It is not enabled in production builds. With the motor supply disconnected, compile and upload:

```sh
arduino-cli compile --fqbn arduino:avr:uno \
  --build-property compiler.cpp.extra_flags="-DGYROGUARD_TEST_WATCHDOG_RESET" \
  firmware/GyroGuard
```

The first boot sets the armed marker and allows a 15 ms watchdog reset. The following boot should detect the marker, print the startup fault and leave STBY low. While the bootloader is waiting for upload, flash the normal production build without the macro; a normal reboot with the test macro still compiled will deliberately trigger the test again.

Do not run this test with motor power connected or the robot standing unsupported. If the test repeatedly resets, the installed bootloader did not preserve the marker as expected; stop and investigate rather than assuming reset recovery works.

## Validation status

GitHub Actions tests the reset-policy helper and marker integrity, and compiles the Uno plus classic Nano new- and old-bootloader targets. Physical reset-marker retention and STBY behavior still require the bench-only hardware test above.
