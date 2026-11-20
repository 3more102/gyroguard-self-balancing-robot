# GyroGuard — Arduino Self-Balancing Obstacle-Avoiding Robot

GyroGuard is a **two-wheel, differential-drive self-balancing robot** for embedded-systems and control-engineering study. It targets the Arduino Uno R3 / classic Nano (ATmega328P, 16 MHz).

## Included

- 200 Hz MPU-6050 attitude loop with complementary filtering
- MPU-6050 identity check, stationary-window startup calibration validation and bounded I²C transactions
- PID input guards reject NaN/infinite sensor or timing values and reset controller state safely
- Inner pitch controller with gyro-rate damping and output limiting
- 50 Hz wheel-encoder velocity loop (outer loop)
- Non-blocking HC-SR04 ranging using an AVR pin-change interrupt
- SG90 scanning servo and deterministic obstacle-avoidance state machine
- Fail-safe range gating: unknown, invalid or stale sensor data never authorizes forward motion or an unverified turn
- TB6612FNG dual H-bridge driver
- Explicit balance arming with live-IMU, upright-angle and low-pitch-rate interlocks
- Watchdog reset recovery and tilt, sensor, timing safety handling
- Serial commands and commissioning documentation

> **Safety:** This is an engineering project, not a ready-to-drive appliance. Initial gains and mechanical parameters are starting points only. A self-balancing robot can move unexpectedly and fall. Commission it with the wheels raised, then use a tether or soft catch area. Keep hands, cables and loose clothing away from wheels.

## Mechanical configuration

Exactly **two driven wheels**, one on each side, on the same axle. No caster or third support wheel is used. The robot balances dynamically; it cannot stand upright with power off.

## Hardware baseline

| Part | Recommended baseline |
|---|---|
| Controller | Arduino Uno R3 or classic Nano, ATmega328P |
| IMU | MPU-6050 / GY-521 breakout |
| Motors | Two matched geared DC motors with quadrature encoders |
| Motor driver | TB6612FNG dual H-bridge |
| Range sensor | HC-SR04 |
| Scanner | SG90 micro servo |
| Battery | Protected 2-cell Li-ion pack (7.4 V nominal), sized for motor current |
| Logic supply | Regulated 5 V buck converter for Uno and sensors |
| Chassis | Rigid two-wheel frame; IMU mounted close to the axle |

Choose motors whose stall current is within the driver and battery limits. The TB6612FNG is not suitable for motors whose stall current exceeds the driver's thermal/current capability. Verify the exact breakout-board rating and motor datasheet before connecting power.

## Repository layout

    firmware/GyroGuard/
      GyroGuard.ino
      config.h
      arm_guard.h
      pid.h / pid.cpp
      watchdog_startup.cpp / watchdog_startup.h
      range_guard.h
      imu_mpu6050.h / imu_mpu6050.cpp
      imu_validation.h
      encoder.h / encoder.cpp
      encoder_math.h
      motor_driver.h / motor_driver.cpp
      sonar.h / sonar.cpp
      navigation_policy.h
      obstacle_avoidance.h / obstacle_avoidance.cpp
    docs/
      HARDWARE.md
      COMMISSIONING.md
      RESOURCE_BUDGETS.md
      WATCHDOG.md
    tests/
      fakes/Arduino.h / fakes/Servo.h
      test_pid.cpp
      test_navigation_policy.cpp
      test_range_guard.cpp
      test_imu_validation.cpp
      test_navigation_state_machine.cpp
      test_servo_center_settle.cpp
      test_balance_arming.cpp
    .github/workflows/arduino-ci.yml

## Build

1. Install Arduino IDE 2.x or Arduino CLI.
2. Open firmware/GyroGuard/GyroGuard.ino.
3. Select **Arduino Uno** (or classic Nano with ATmega328P).
4. Compile and upload. The sketch uses Wire from the AVR core and the Servo library. Install Servo through Arduino IDE Library Manager or run arduino-cli lib install Servo.
5. Read the hardware and commissioning guides before applying motor power.

Arduino CLI example:

    arduino-cli core update-index
    arduino-cli core install arduino:avr
    arduino-cli lib install Servo
    arduino-cli compile --fqbn arduino:avr:uno firmware/GyroGuard
    arduino-cli upload -p YOUR_PORT --fqbn arduino:avr:uno firmware/GyroGuard

Host unit tests (requires a C++11 compiler):

    g++ -std=c++11 -Wall -Wextra -Werror tests/test_pid.cpp firmware/GyroGuard/pid.cpp -o /tmp/gyroguard-pid-test
    /tmp/gyroguard-pid-test
    g++ -std=c++11 -Wall -Wextra -Werror tests/test_navigation_policy.cpp -o /tmp/gyroguard-navigation-test
    /tmp/gyroguard-navigation-test
    g++ -std=c++11 -Wall -Wextra -Werror tests/test_range_guard.cpp -o /tmp/gyroguard-range-guard-test
    /tmp/gyroguard-range-guard-test
    g++ -std=c++11 -Wall -Wextra -Werror tests/test_imu_validation.cpp -o /tmp/gyroguard-imu-validation-test
    /tmp/gyroguard-imu-validation-test
    g++ -std=c++11 -Wall -Wextra -Werror -I tests/fakes tests/test_navigation_state_machine.cpp firmware/GyroGuard/obstacle_avoidance.cpp -o /tmp/gyroguard-navigation-state-machine-test
    /tmp/gyroguard-navigation-state-machine-test
    g++ -std=c++11 -Wall -Wextra -Werror -I tests/fakes tests/test_servo_center_settle.cpp firmware/GyroGuard/obstacle_avoidance.cpp -o /tmp/gyroguard-servo-settle-test
    /tmp/gyroguard-servo-settle-test
    g++ -std=c++11 -Wall -Wextra -Werror tests/test_balance_arming.cpp -o /tmp/gyroguard-arm-test
    /tmp/gyroguard-arm-test

CI runs all host test suites and compiles the Uno and classic Nano targets. It enforces project budgets of **24,000 bytes of flash** and **1,200 bytes of static SRAM** per target. The SRAM budget reserves at least 848 bytes for runtime stack and other dynamic use on the ATmega328P; passing the static budget is not a substitute for hardware stack and stress testing. See [RESOURCE_BUDGETS.md](docs/RESOURCE_BUDGETS.md).

## First-use behavior

- Motor control and autonomous movement are **disabled on boot**.
- The firmware verifies the MPU-6050 identity and rejects unstable or gravity-inconsistent startup calibration samples.
- MPU-6050 I²C transactions have a 5 ms timeout; a failed transaction causes a latched motor-off fault.
- The robot calibrates the IMU while it is held upright and stationary.
- After calibration, motor outputs remain disabled.
- Wait for valid pitch telemetry, hold the robot within 8 degrees of upright and nearly motionless (pitch rate below 10 degrees/s), then send 'b' over Serial Monitor (115200 baud) to arm balancing. The pitch sample must be no older than 15 ms.
- Send 'a' only after balance-only operation is stable to enable autonomous cruise and obstacle avoidance.
- Autonomous forward travel requires a fresh, valid center-range measurement. Invalid or stale data holds translation at zero while balancing remains active.
- After an avoidance turn, a fresh forward measurement is required before cruise resumes.
- If neither side has verified clearance, the robot holds position and retries sensing; it does not reverse without rear sensing.
- Send 's' to stop autonomous travel while keeping balance active.
- Send 'd' to disarm motor outputs; the robot will fall unless physically supported.
- Send 't' to toggle telemetry; send '?' for commands.
- A tilt, sensor or control-timing fault latches the motor driver off. Power-cycle after correcting the cause.

Do not enable autonomous mode until balance-only operation has been tuned and verified.

## Control architecture

1. **Inner loop — 200 Hz:** MPU-6050 accelerometer + gyroscope complementary filter; pitch controller commands common motor effort.
2. **Outer loop — 50 Hz:** encoder-derived linear velocity controller requests a small pitch setpoint offset.
3. **Navigation:** the HC-SR04 is scanned left/center/right by the servo. The avoidance state machine changes the velocity request and applies a differential turn bias. It never pauses the balance loop. Unknown, invalid or stale range readings inhibit translation instead of being interpreted as clear space. No reverse is commanded without rear sensing.

The serial telemetry includes pitch, gyro rate, wheel speed, target angle, motor command, forward sonar range and balance-armed state. The initial gains in config.h are starting values, not universal tuning values. Motor polarity, encoder polarity, wheel diameter, encoder counts and balance gains must be verified on the assembled robot.

## Safety and limitations

- The ultrasonic sensor is a simple single-beam rangefinder; soft, angled, narrow or sound-absorbing objects may be missed.
- A single forward scanner does not provide full 360-degree perception or rear coverage.
- If neither side scan verifies sufficient clearance, the robot holds position and retries rather than reversing into an unobserved area.
- The avoidance turn is a timed differential-drive maneuver; it is not SLAM or map-based navigation.
- No software can compensate for an underpowered driver, loose frame, poor wheel alignment, unsuitable motors or an incorrectly mounted IMU.
- Use a physical power switch and accessible battery disconnect.

## License

MIT. See LICENSE.
