# GyroGuard Validation Record

A passing software build does not prove that the physical robot balances, stops safely or has electrically compatible components.

## Software validation

The repository CI is configured to:
- Run PID controller host tests.
- Run obstacle turn-selection policy tests, including the no-rear-sensor blocked-path case.
- Run IMU calibration quality tests for stationary, moving, invalid-gravity and insufficient-sample inputs.
- Run front-range freshness tests, including `millis()` rollover.
- Compile for Arduino Uno and classic Nano.
- Enforce 24,000-byte flash and 1,200-byte static-SRAM project budgets.

Check the latest GitHub Actions result for the exact commit before treating these checks as passed.

## Physical validation status

< NOT VALIDATED >

No physical robot behavior is claimed as verified by the repository or CI alone. The following must be completed with the actual components:

1. Record motor model, rated voltage, gearbox ratio, stall current and encoder specification.
2. Verify the TB6612FNG breakout's thermal/current capability against motor startup and stall current.
3. Verify supply rails, fuse, physical disconnect, common ground and servo buck capacity.
4. Confirm IMU axes and pitch sign with motor power disconnected.
5. Confirm motor and encoder polarity with wheels raised.
6. Demonstrate balance-only operation in a tethered, clear test area before tuning autonomous motion.
7. Verify HC-SR04 distances against measured targets and confirm servo left/right orientation.
8. Test obstacle stop, side selection, blocked hold and fresh-center-range recovery at low speed.
9. Record the firmware commit, battery voltage under load, measured wheel diameter, encoder counts/revolution and final gains.

## Safety boundary

The robot has forward ultrasonic sensing only. It does not have rear coverage, so the navigation policy must not command blind reverse motion. Unknown, invalid or stale range data must not authorize forward travel. Keep autonomous travel disabled until the hardware commissioning record is complete.
