# AVR Build and Resource Budgets

GyroGuard targets the ATmega328P on the Arduino Uno R3 and classic Nano. The device has 32 KB of flash and 2 KB of SRAM; the bootloader reduces the flash available to the sketch. SRAM is shared by static data, library/runtime state, heap allocations and the call stack.

## CI targets

The Arduino CI workflow validates both supported board definitions:

- `arduino:avr:uno`
- `arduino:avr:nano`

Both builds use the same firmware directory and install the AVR core plus the Arduino Servo library.

## Project guardrails

| Resource | CI ceiling | Reason |
|---|---:|---|
| Sketch flash | 24,000 bytes | Retains meaningful program-space headroom for future control and diagnostics work |
| Static SRAM | 1,200 bytes | Leaves at least 848 bytes of the ATmega328P's 2,048-byte SRAM for stack and other runtime use |

These are project limits, not the microcontroller's hardware limits. The CI job parses the memory summary emitted by Arduino CLI and fails if it cannot find the report or if either ceiling is exceeded.

## Baseline

The successful build of main at commit `d40bcc8b2e46b59965ea8465717391cdeca723ac` reported for Arduino Uno:

- Flash: 14,938 bytes
- Static/global SRAM: 624 bytes
- SRAM remaining after static allocation: 1,424 bytes

The CI workflow introduced with this document adds the classic Nano build; its result must be checked in the corresponding workflow run rather than inferred from the Uno result.

## What this check does not prove

A passing static SRAM budget does **not** measure peak stack depth, interrupt stack usage, heap fragmentation, timing jitter, motor stability, sensor reliability or electrical integrity. Those require runtime instrumentation and physical commissioning.

Before autonomous operation:

1. Follow [HARDWARE.md](HARDWARE.md) and [COMMISSIONING.md](COMMISSIONING.md).
2. Keep the wheels raised for initial motor and sensor checks.
3. Tune balance-only behavior before enabling obstacle avoidance.
4. Repeat tests under low battery, motor load and serial telemetry activity.
5. Treat a successful compile as a software build result, not evidence that the physical robot is safe or stable.
