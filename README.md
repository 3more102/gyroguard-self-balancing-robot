# GyroGuard — Arduino Self-Balancing Obstacle-Avoiding Robot

GyroGuard is a **two-wheel, differential-drive self-balancing robot** for embedded-systems and control-engineering study. It targets the Arduino Uno R3 / classic Nano (ATmega328P, 16 MHz).

## Included

- 200 Hz MPU-6050 attitude loop with complementary filtering
- Inner pitch controller with gyro-rate damping and output limiting
- 50 Hz wheel-encoder velocity loop (outer loop)
- Non-blocking HC-SR04 ranging using an AVR pin-change interrupt
- SG90 scanning servo and deterministic obstacle-avoidance state machine
- Fail-safe range gating: unknown sensor data never authorizes forward motion or an unverified turn
- No blind reverse commands; if neither side is proven clear, translation remains stopped
- TB6612FNG dual H-bridge driver
- Tilt, sensor, timing and startup safety handling
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
      pid.h / pid.cpp
      imu_mpu6050.h / imu_mpu6050.cpp
      encoder.h / encoder.cpp
      motor_driver.h / motor_driver.cpp
      sonar.h / sonar.cpp
      navigation_policy.h
      obstacle_avoidance.h / obstacle_avoidance.cpp
    docs/
      HARDWARE.md
      COMMISSIONING.md
      RESOURCE_BUDGETS.md
    tests/
      test_pid.cpp
      test_navigation_policy.cpp
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

CI runs both host test suites and compiles the Uno and classic Nano targets. It enforces project budgets of **24,000 bytes of flash** and **1,200 bytes of static SRAM** per target. The SRAM budget reserves at least 848 bytes for runtime stack and other dynamic use on the ATmega328P; passing the static budget is not a substitute for hardware stack and stress testing. See [RESOURCE_BUDGETS.md](docs/RESOURCE_BUDGETS.md).

## First-use behavior

- Autonomous movement is **disabled on boot**.
- The robot calibrates the IMU while it is held upright and stationary.
- Once calibration succeeds, the balance loop is enabled.
- Send 'a' over Serial Monitor (115200 baud) to enable autonomous cruise and obstacle avoidance.
- Autonomous forward travel waits for the center servo to settle and for a valid range at or beyond OBSTACLE_CLEAR_MM. Invalid or marginal readings hold translation at zero while balancing remains active.
- After an avoidance turn, the servo settles and a fresh clear forward measurement is required before cruise resumes.
- If both side scans are blocked or uncertain, the robot holds position. It does not reverse without rear-facing sensing.
- Send 's' to request zero forward speed while keeping balance active.
- Send 't' to toggle telemetry; send '?' for commands.
- A tilt, sensor or control-timing fault latches the motor driver off. Power-cycle after correcting the cause.

Do not enable autonomous mode until balance-only operation has been tuned and verified.

## Control architecture

1. **Inner loop — 200 Hz:** MPU-6050 accelerometer + gyroscope complementary filter; pitch controller commands common motor effort.
2. **Outer loop — 50 Hz:** encoder-derived linear velocity controller requests a small pitch setpoint offset.
3. **Navigation:** the HC-SR04 is scanned left/center/right by the servo. A side is selectable only with a valid reading at or beyond OBSTACLE_CLEAR_MM. If neither side is proven clear, translation stays at zero. The balance loop continues during all scans and holds.

The serial telemetry includes pitch, gyro rate, wheel speed, target angle, motor command and forward sonar range. The initial gains in config.h are starting values, not universal tuning values. Motor polarity, encoder polarity, wheel diameter, encoder counts and balance gains must be verified on the assembled robot.

## Safety and limitations

- The ultrasonic sensor is a simple single-beam rangefinder; soft, angled, narrow or sound-absorbing objects may be missed.
- A single forward scanner does not provide full 360-degree perception or rear clearance; GyroGuard therefore never reverses blindly.
- The avoidance turn is a timed differential-drive maneuver; it is not SLAM or map-based navigation.
- No software can compensate for an underpowered driver, loose frame, poor wheel alignment, unsuitable motors or an incorrectly mounted IMU.
- Use a physical power switch and accessible battery disconnect.

## License

MIT. See LICENSE.
