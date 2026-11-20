# GyroGuard Commissioning and Tuning

Follow these steps in order. Do not begin autonomous obstacle avoidance until the robot can balance reliably.

## Stage 1 — Inspect and compile

1. Assemble the two-wheel chassis rigidly; align both wheels and the axle.
2. Complete the wiring in HARDWARE.md, including the 10 kΩ TB6612 STBY pull-down and local driver decoupling.
3. With the battery disconnected, check all power rails and grounds.
4. Compile for Arduino Uno. Upload with the motor battery disconnected.
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
5. Send 'a' only after balance-only operation is stable. The robot waits for a fresh, valid forward measurement before moving.
6. Confirm that an invalid or stale forward reading holds travel at zero while the balance controller remains active. A sensor that repeatedly reports -1 will intentionally prevent autonomous travel; check wiring, alignment, target reflectivity and the configured range.
7. Place obstacles on both sides and verify the robot does not turn toward a side whose clearance has not been measured. If neither side has verified clearance, it holds position and retries sensing; it does not reverse without rear sensing.
8. After a scan or turn, verify the center servo has settled and a fresh center measurement confirms sufficient clearance before cruise resumes.
9. Adjust OBSTACLE_STOP_MM, OBSTACLE_CLEAR_MM, SERVO_SETTLE_MS, TURN_DURATION_MS and TURN_BIAS_PWM only after confirming sensor readings and motor polarity.

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

An invalid or stale ultrasonic measurement inhibits autonomous translation while the balance loop remains active and the sensor is retried. If neither scan direction verifies sufficient clearance, the robot holds position instead of reversing without rear sensing. The servo is allowed to settle before a new center measurement is accepted. After a latched control fault, inspect the robot, correct the cause, and power-cycle. A reset is intentional; do not automatically re-arm a robot that has fallen.

## Tuning record

Record each hardware change and gain set. Record battery voltage under load, motor/gearbox model, wheel diameter, encoder counts per wheel revolution, IMU mounting orientation, balance gains, velocity gains, drift, obstacle stop distance and turn duration.
