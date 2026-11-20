# GyroGuard Commissioning and Tuning

Follow these steps in order. Do not begin autonomous obstacle avoidance until the robot can balance reliably.

## Stage 1 — Inspect and compile

1. Assemble the two-wheel chassis rigidly; align both wheels and the axle.
2. Complete the wiring in HARDWARE.md, including the 10 kΩ TB6612 STBY pull-down, local driver decoupling and protected-2S battery divider to A2.
3. With the battery disconnected, check all power rails and grounds.
4. Compile for Arduino Uno. Upload with the motor battery disconnected.
5. Open Serial Monitor at 115200 baud. Compare the separate batteryMv/valid/low record with a multimeter at the pack terminals; adjust BATTERY_ADC_REFERENCE_MV to the measured 5 V ADC reference if needed.

## Stage 2 — Verify sensor identity and orientation

1. Keep motor power disconnected or wheels safely raised.
2. Hold the robot upright and completely still during startup calibration (about 3 seconds).
3. The firmware checks the MPU-6050 WHO_AM_I register before configuring the sensor.
4. The calibration window is accepted only when the mean pitch offset is within 15 degrees of upright, accelerometer-angle variation is low, the three-axis gyro rate is small and gravity magnitude remains plausible.
5. If calibration fails, check the I2C wiring/address, secure the IMU mount, place the robot upright, and repeat calibration without moving it.
6. Watch the reported pitch telemetry.
7. Tilt the robot's top toward the front: pitch should increase. Tilt it backward: pitch should decrease.
8. If not, correct the IMU orientation/axis mapping before continuing.

The calibration thresholds in config.h are conservative firmware acceptance limits, not MPU-6050 datasheet accuracy specifications. A stable but substantially tilted pose or a moving sensor must not be accepted as the zero reference. Do not loosen the limits to hide mechanical vibration.

## Stage 3 — Verify motor polarity with wheels raised

The balance controller assumes **positive motor command moves both wheels forward**.

Before attempting to balance, temporarily test each motor at low PWM using a safe test sketch or current-limited bench supply. Confirm positive command makes both wheels propel the robot forward. If one wheel is reversed, change its LEFT_MOTOR_SIGN or RIGHT_MOTOR_SIGN in config.h.

Also verify encoder signs: roll the robot forward by hand. Both encoder counts should increase and measured average speed should be positive. If one is negative, change that side's *_ENCODER_SIGN in config.h.

During autonomous straight travel, both encoders must report movement at least once every 750 ms. During a differential turn, at least one encoder must report movement every 400 ms. A timeout inhibits autonomous travel and returns the velocity target to upright while balance remains active. The encoder fault is latched until power-cycle; inspect wiring, polarity, wheel obstruction and motor operation before resetting. These thresholds are initial guardrails and must be checked with the actual motor/encoder combination.

Do not infer polarity from wire colors; motor and encoder wiring varies.

## Stage 4 — Balance-only test

1. Keep autonomous mode disabled (default); motor outputs remain disabled after calibration.
2. Use a soft, clear test area with a safety tether or helper ready to catch the robot.
3. With wheels raised, use a temporary low-PWM motor test sketch to verify motor and encoder polarity.
4. Put the wheels on the ground while holding the frame within 8 degrees of upright and nearly motionless (pitch rate below 10 degrees/s). Wait for valid pitch telemetry no older than 15 ms, then send 'b' to arm balance.
5. Keep a hand near the physical power switch and release the frame only into the tethered test area.
6. If it accelerates in the direction of the fall, immediately cut power. Reverse the balance sign / motor polarity only after verifying IMU sign and wheel direction.
7. Tune the inner balance loop first:
   - Increase BALANCE_KP gradually until it responds strongly but begins to oscillate.
   - Back off slightly.
   - Increase BALANCE_KD to damp oscillation.
   - Keep BALANCE_KI at zero initially. Add only a small amount if a persistent bias remains after mechanical/IMU calibration.
8. Tune the outer velocity loop after stable upright balancing:
   - Start with low VELOCITY_KP and VELOCITY_KI.
   - If it slowly drifts, verify encoder polarity and counts/revolution before changing gains.
   - Keep MAX_LEAN_DEG small while commissioning.

There is no universal stable PID setting. Mass, centre of gravity, motor torque, gear ratio, wheel diameter, battery voltage, frame stiffness, motor deadband and sample timing all matter.

## Stage 5 — Verify obstacle sensing

1. Keep the robot supported and autonomous mode disabled.
2. Verify the servo points forward at 90 degrees.
3. Place a flat obstacle at known distances (for example 20 cm, 40 cm and 80 cm). A nonzero echo shorter than the configured 30 mm reliable range is reported as a 0 mm obstruction; it is not a precision near-field measurement.
4. With autonomous mode still disabled, verify the rangeMm field in Serial Monitor updates with plausible distances after the center servo has settled; -1 means no valid echo was received. This is diagnostic telemetry only and must not command translation.
5. Send 'a' only after balance-only operation is stable and balance is armed. The center servo must settle for SERVO_SETTLE_MS, and the robot will not drive autonomously until it has a fresh, valid forward-range measurement at or beyond OBSTACLE_CLEAR_MM.
6. Confirm that an invalid or stale forward reading holds travel at zero while the balance controller remains active. A sensor that repeatedly reports -1 will intentionally prevent autonomous travel; check wiring, alignment, target reflectivity and the configured range.
7. Place obstacles on both sides and verify the robot does not turn toward a side whose clearance has not been measured. If neither side has verified clearance, it holds position and retries; it does not reverse without rear sensing.
8. After a scan, re-arm or turn, wait SERVO_SETTLE_MS for the center servo to settle; forward travel remains disabled until a fresh center measurement confirms clearance.
9. Adjust OBSTACLE_STOP_MM, OBSTACLE_CLEAR_MM, SERVO_SETTLE_MS, TURN_DURATION_MS and TURN_BIAS_PWM only after confirming sensor readings and motor polarity.


## Stage 6 — Verify low-battery travel inhibit

1. Do not deeply discharge the Li-ion pack to test the threshold.
2. With the wheels raised and a current-limited adjustable bench supply replacing the pack, lower the pack input slowly toward 6.8 V while keeping the regulated 5 V logic rail stable.
3. Verify the separate telemetry record reports batteryMv near the multimeter reading and valid=1. The low flag is driven by the raw sample (not the smoothed display value) and must assert on the first 100 ms sample at/below 6.8 V. With balance armed, sending 'a' must leave autonomous mode disabled. If autonomy was already active, the first low sample must disable travel while the balance loop remains active.
4. Raise the supply to at least 7.0 V and wait for the filtered reading to recover. The low flag clears, but autonomous travel stays disabled until 'a' is deliberately sent again.
5. With the pack voltage safe, disconnect only the divider's switched-positive sense lead while leaving the lower 10 kΩ resistor connected to ground. Verify valid=0 and that 'a' remains blocked.
6. Restore the sense lead, verify the reading against a multimeter, then reconnect the protected 2S pack.

The battery monitor is a travel inhibit, not a motor power cut or cell-level BMS. Do not use it to justify deep discharge or omit the pack's protection circuit.

## Serial commands
## Stage 7 — Verify armed-reset recovery (bench only)

1. Disconnect the motor supply completely. Keep the robot physically supported with wheels raised; power only the controller over USB.
2. Build a temporary Uno test image with `GYROGUARD_TEST_WATCHDOG_RESET` enabled, following the command in WATCHDOG.md. The test marks the balance state as armed and deliberately lets the watchdog expire before normal IMU calibration.
3. On the next boot, verify the firmware reports an unexpected reset while balance was armed, latches a startup fault, and keeps TB6612 STBY low.
4. While the bootloader is waiting for upload, flash the normal production image without the test macro. Do not expect a normal application boot while the test macro remains compiled.
5. If the test build repeatedly resets, the installed bootloader did not preserve the SRAM marker through its startup path. Stop the test and do not rely on this marker with that bootloader until the startup path is corrected.

**Never perform this test with motor power connected or while the robot is standing unsupported.**


| Command | Action |
|---|---|
| b | Arm balance using an IMU sample no older than 15 ms, within 8 degrees of upright and below 10 degrees/s pitch rate |
| d | Disarm motor outputs; robot will fall unless physically supported |
| a | Enable autonomous cruise and obstacle avoidance (only while balance is armed) |
| s | Disable travel; keep balance loop active |
| t | Toggle telemetry |
| ? | Print command help |

## Fault behavior

The motor driver is disabled and the fault is latched if:
- MPU-6050 identity verification fails, an I²C transaction times out, or an IMU runtime read fails
- startup calibration is unstable or the sensor identity check fails
- absolute pitch exceeds MAX_SAFE_TILT_DEG
- the control-loop gap exceeds MAX_CONTROL_GAP_US

An invalid or stale ultrasonic measurement inhibits autonomous translation while the balance loop remains active and the sensor is retried. If neither scan direction verifies sufficient clearance, the robot holds position instead of reversing without rear sensing. A low or invalid battery reading inhibits autonomous travel while keeping balance control active; after voltage recovery, send 'a' again deliberately. After a latched control fault, inspect the robot, correct the cause, and power-cycle. A reset is intentional; do not automatically re-arm a robot that has fallen. After any normal boot or watchdog reset, balance remains disarmed until 'b' is deliberately sent. The 'd' command removes motor torque; use it only while the robot is physically supported. Use the battery switch for an emergency power cut.

## Tuning record

Record each hardware change and gain set. Record battery voltage under load, motor/gearbox model, wheel diameter, encoder counts per wheel revolution, IMU mounting orientation, balance gains, velocity gains, drift, obstacle stop distance and turn duration.
