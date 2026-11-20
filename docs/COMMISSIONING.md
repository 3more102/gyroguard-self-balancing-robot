# GyroGuard Commissioning and Tuning

Follow these steps in order. Do not begin autonomous obstacle avoidance until the robot can balance reliably.

## Stage 1 — Inspect and compile

1. Assemble the two-wheel chassis rigidly; align both wheels and the axle.
2. Complete the wiring in HARDWARE.md.
3. With the battery disconnected, check all power rails and grounds.
4. Compile for Arduino Uno or classic Nano. Upload with the motor battery disconnected.
5. Open Serial Monitor at 115200 baud.

## Stage 2 — Verify sensor orientation

1. Keep motor power disconnected or wheels safely raised.
2. Hold the robot upright and motionless during startup calibration (about 3 seconds).
3. Watch the reported pitch telemetry.
4. Tilt the robot's top toward the front: pitch should increase. Tilt it backward: pitch should decrease.
5. If not, correct the IMU orientation/axis mapping before continuing.

The calibration assumes the chassis is held at its intended upright angle. The IMU must not move during the calibration window.

## Stage 3 — Verify motor polarity with wheels raised

The balance controller assumes **positive motor command moves both wheels forward**.

Before attempting to balance, temporarily test each motor at low PWM using a safe test sketch or current-limited bench supply. Confirm positive command makes both wheels propel the robot forward. If one wheel is reversed, change its LEFT_MOTOR_SIGN or RIGHT_MOTOR_SIGN in config.h.

Also verify encoder signs: roll the robot forward by hand. Both encoder counts should increase and measured average speed should be positive. If one is negative, change that side's *_ENCODER_SIGN in config.h.

Do not infer polarity from wire colors; motor and encoder wiring varies.

## Stage 4 — Balance-only test

1. Keep autonomous mode disabled (default).
2. Use a soft, clear test area with a safety tether or helper ready to catch the robot.
3. Start with wheels raised to verify motor effort reacts against a small tilt. Positive forward tilt should cause positive forward wheel effort.
4. Put the wheels on the ground while holding the frame upright. Keep a hand near the power switch.
5. If it accelerates in the direction of the fall, immediately cut power. Reverse the balance sign / motor polarity only after verifying IMU sign and wheel direction.
6. Tune the inner balance loop first:
   - Increase BALANCE_KP gradually until it responds strongly but begins to oscillate.
   - Back off slightly.
   - Increase BALANCE_KD to damp oscillation.
   - Keep BALANCE_KI at zero initially. Add only a small amount if a persistent bias remains after mechanical/IMU calibration.
7. Tune the outer velocity loop after stable upright balancing:
   - Start with low VELOCITY_KP and VELOCITY_KI.
   - If it slowly drifts, verify encoder polarity and counts/revolution before changing gains.
   - Keep MAX_LEAN_DEG small while commissioning.

There is no universal stable PID setting. Mass, centre of gravity, motor torque, gear ratio, wheel diameter, battery voltage, frame stiffness, motor deadband and sample timing all matter.

## Stage 5 — Verify obstacle sensing

1. Keep the robot supported and autonomous mode disabled.
2. Verify the servo points forward at 90 degrees.
3. Place a flat obstacle at known distances (for example 20 cm, 40 cm and 80 cm).
4. Verify the rangeMm field in Serial Monitor reports plausible distances; -1 means no valid echo was received.
5. Send a only after balance-only operation is stable. The robot will not drive autonomously until it has a valid forward-range measurement.
6. Confirm that an invalid forward reading holds travel at zero while the balance controller remains active. A sensor that repeatedly reports -1 will intentionally prevent autonomous travel; check wiring, alignment, target reflectivity and the configured range.
7. Place obstacles on both sides and verify the robot does not turn toward a side whose clearance has not been measured. If neither side has verified clearance, it holds position and retries.
8. After a turn, forward travel remains disabled until a fresh center measurement confirms clearance.
9. Adjust OBSTACLE_STOP_MM, OBSTACLE_CLEAR_MM, TURN_DURATION_MS and TURN_BIAS_PWM only after confirming sensor readings and motor polarity.

## Serial commands

| Command | Action |
|---|---|
| a | Enable autonomous cruise and obstacle avoidance |
| s | Disable travel; keep balance loop active |
| t | Toggle telemetry |
| ? | Print command help |

## Fault behavior

The motor driver is disabled and the fault is latched if:
- IMU initialization or a runtime read fails
- absolute pitch exceeds MAX_SAFE_TILT_DEG
- the control-loop gap exceeds MAX_CONTROL_GAP_US

An invalid ultrasonic measurement is handled separately: autonomous translation is held at zero, but the balance loop remains active while the sensor is retried. A latched control fault requires inspection and a power-cycle; do not automatically re-arm a robot that has fallen.

**Watchdog / brown-out recovery:** GyroGuard captures Optiboot's reset flags from register r2 before normal C initialization and also captures MCUSR for bootloaders that preserve it. The watchdog is disabled in the early startup section. An application watchdog reset (WDRF without EXTRF) or a brown-out reset keeps the motor driver disabled and latches a fault instead of automatically restarting balance control. Optiboot may report WDRF together with EXTRF after its own upload timeout; that combination is not treated as an application watchdog failure.

For Arduino Nano boards, use the ATmega328P (new bootloader / Optiboot) setting when relying on watchdog recovery. The old ATmegaBOOT bootloader can reset-loop after a watchdog reset before the sketch regains control; compiling for the old Nano target does not verify that bootloader's runtime recovery behavior. Inspect the robot and power system, then power-cycle while holding the chassis upright for calibration.

## Tuning record

Record each hardware change and gain set. Record battery voltage under load, motor/gearbox model, wheel diameter, encoder counts per wheel revolution, IMU mounting orientation, balance gains, velocity gains, drift, obstacle stop distance and turn duration.
