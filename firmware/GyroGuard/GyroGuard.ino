#include <Arduino.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include "config.h"
#include "pid.h"
#include "imu_mpu6050.h"
#include "encoder.h"
#include "motor_driver.h"
#include "obstacle_avoidance.h"
#include <math.h>

// Optiboot clears MCUSR but passes the original reset flags in r2.
// Legacy bootloaders may leave the flags in MCUSR instead.
uint8_t gyroGuardBootloaderResetCause __attribute__((section(".noinit")));
uint8_t gyroGuardMcusrResetCause __attribute__((section(".noinit")));

void gyroGuardCaptureBootloaderResetCause(void)
    __attribute__((used, naked, section(".init0")));

void gyroGuardCaptureBootloaderResetCause(void) {
  __asm__ __volatile__("sts %0, r2\n" : "=m" (gyroGuardBootloaderResetCause));
}

// Disable the AVR watchdog before normal Arduino initialization. This also
// prevents a watchdog-reset loop on bootloaders that leave WDT running.
void gyroGuardCaptureMcusrAndDisableWatchdog(void)
    __attribute__((used, unused, naked, section(".init3")));

void gyroGuardCaptureMcusrAndDisableWatchdog(void) {
  gyroGuardMcusrResetCause = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

namespace {
MPU6050 imu;
EncoderPair encoders;
MotorDriver motors;
ObstacleAvoidance navigation;

PIDController balancePid(BALANCE_KP, BALANCE_KI, BALANCE_KD,
                         -BALANCE_OUTPUT_LIMIT, BALANCE_OUTPUT_LIMIT);
PIDController velocityPid(VELOCITY_KP, VELOCITY_KI, 0.0f,
                          -MAX_LEAN_DEG, MAX_LEAN_DEG);

float gyroBiasDps = 0.0f;
float imuZeroAngleDeg = 0.0f;
float pitchDeg = 0.0f;
float pitchRateDps = 0.0f;
float measuredSpeedMmS = 0.0f;
float targetAngleDeg = 0.0f;
bool faultLatched = false;
bool telemetryEnabled = true;

uint32_t lastControlUs = 0;
uint32_t lastVelocityUs = 0;
uint32_t lastTelemetryMs = 0;
float velocityLoopDt = 0.02f;
EncoderSnapshot previousCounts{0, 0};

void latchFault(const __FlashStringHelper *reason) {
  const bool firstFault = !faultLatched;
  faultLatched = true;

  // Disable the H-bridge before any potentially blocking serial output.
  motors.disable();

  if (firstFault) {
    Serial.print(F("FAULT: "));
    Serial.println(reason);
  }
}

bool updateVelocity(uint32_t nowUs) {
  const uint32_t elapsedUs = nowUs - lastVelocityUs;
  if (elapsedUs < VELOCITY_PERIOD_US) return false;

  const EncoderSnapshot current = encoders.snapshot();
  const long leftDelta = current.left - previousCounts.left;
  const long rightDelta = current.right - previousCounts.right;
  previousCounts = current;

  velocityLoopDt = static_cast<float>(elapsedUs) * 1.0e-6f;
  const float averageCounts = 0.5f *
      (static_cast<float>(leftDelta) + static_cast<float>(rightDelta));
  const float distanceMm = averageCounts * WHEEL_CIRCUMFERENCE_MM /
                           ENCODER_COUNTS_PER_WHEEL_REV;
  measuredSpeedMmS = distanceMm / velocityLoopDt;
  lastVelocityUs = nowUs;
  return true;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'a' || command == 'A') {
      if (!faultLatched) {
        navigation.setAutonomous(true);
        velocityPid.reset();
        Serial.println(F("Autonomous mode enabled."));
      }
    } else if (command == 's' || command == 'S') {
      navigation.setAutonomous(false);
      velocityPid.reset();
      Serial.println(F("Autonomous mode disabled; balancing remains active."));
    } else if (command == 't' || command == 'T') {
      telemetryEnabled = !telemetryEnabled;
    } else if (command == '?') {
      Serial.println(F("Commands: a=autonomous, s=stop travel, t=telemetry, ?=help"));
    }
  }
}

void printTelemetry(uint32_t nowMs, float motorCommand) {
  if (!telemetryEnabled ||
      static_cast<uint32_t>(nowMs - lastTelemetryMs) < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMs = nowMs;

  Serial.print(F("pitch=")); Serial.print(pitchDeg, 2);
  Serial.print(F(",rate=")); Serial.print(pitchRateDps, 2);
  Serial.print(F(",speed=")); Serial.print(measuredSpeedMmS, 1);
  Serial.print(F(",targetAngle=")); Serial.print(targetAngleDeg, 2);
  Serial.print(F(",motor=")); Serial.print(motorCommand, 1);
  Serial.print(F(",rangeMm="));
  if (navigation.frontDistanceValid()) Serial.print(navigation.frontDistanceMm());
  else Serial.print(-1);
  Serial.print(F(",auto=")); Serial.print(navigation.autonomous() ? 1 : 0);
  Serial.print(F(",fault=")); Serial.println(faultLatched ? 1 : 0);
}
}

void setup() {
  // Prefer MCUSR when a legacy bootloader preserves it. Optiboot normally
  // clears MCUSR, so use its saved r2 flags only for safety-relevant reset bits.
  uint8_t resetCause = gyroGuardMcusrResetCause;
  if (resetCause == 0) {
    const uint8_t bootloaderSafetyFlags =
        gyroGuardBootloaderResetCause & (_BV(WDRF) | _BV(BORF));
    if (bootloaderSafetyFlags != 0) {
      resetCause = gyroGuardBootloaderResetCause;
    }
  }

  // Put the motor driver in standby before initializing other peripherals.
  motors.begin();
  Serial.begin(SERIAL_BAUD);

  Serial.println(F("\nGyroGuard boot"));
  Serial.print(F("Reset cause flags: 0x"));
  Serial.println(resetCause, HEX);

  // Optiboot can report WDRF+EXTRF after its own upload timeout. That is not
  // an application watchdog failure. An application WDT reset has WDRF set
  // without EXTRF. Any brown-out reset requires inspection.
  const bool watchdogReset = (resetCause & _BV(WDRF)) != 0 &&
                             (resetCause & _BV(EXTRF)) == 0;
  const bool brownOutReset = (resetCause & _BV(BORF)) != 0;
  if (watchdogReset || brownOutReset) {
    if (watchdogReset) {
      Serial.println(F("Previous reset: application watchdog."));
    }
    if (brownOutReset) {
      Serial.println(F("Previous reset: brown-out."));
    }
    latchFault(F("Reset requires inspection; power-cycle to re-arm."));
    return;
  }

  encoders.begin();
  navigation.begin();

  Serial.println(F("Keep robot upright and motionless during IMU calibration."));

  if (!imu.begin()) {
    latchFault(F("MPU6050 not detected / I2C setup failed"));
    return;
  }

  if (!imu.calibrate(IMU_CALIBRATION_SAMPLES, imuZeroAngleDeg, gyroBiasDps)) {
    latchFault(F("IMU calibration read failed"));
    return;
  }

  previousCounts = encoders.snapshot();
  lastControlUs = micros();
  lastVelocityUs = lastControlUs;
  lastTelemetryMs = millis();

  Serial.print(F("IMU zero angle: "));
  Serial.println(imuZeroAngleDeg, 3);
  Serial.print(F("Gyro Y bias dps: "));
  Serial.println(gyroBiasDps, 4);
  Serial.println(F("Balance loop armed. Keep a hand near the power switch."));
  Serial.println(F("Autonomous mode is disabled. Send 'a' only after balance tuning."));

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();
  readSerialCommands();

  if (faultLatched) {
    motors.disable();
    return;
  }

  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;

  const uint32_t elapsedUs = nowUs - lastControlUs;
  lastControlUs = nowUs;
  if (elapsedUs > MAX_CONTROL_GAP_US) {
    latchFault(F("Control-loop timing overrun"));
    return;
  }

  const float dt = static_cast<float>(elapsedUs) * 1.0e-6f;
  if (!imu.readPitch(dt, imuZeroAngleDeg, gyroBiasDps, pitchDeg, pitchRateDps)) {
    latchFault(F("MPU6050 read failed"));
    return;
  }

  if (fabsf(pitchDeg) > MAX_SAFE_TILT_DEG) {
    latchFault(F("Robot exceeded safe tilt angle; power-cycle after inspection"));
    return;
  }

  const NavigationCommand nav = navigation.update(millis(), nowUs);
  if (updateVelocity(nowUs)) {
    float targetSpeed = nav.targetSpeedMmS;
    if (targetSpeed > MAX_TARGET_SPEED_MM_S) targetSpeed = MAX_TARGET_SPEED_MM_S;
    if (targetSpeed < -MAX_TARGET_SPEED_MM_S) targetSpeed = -MAX_TARGET_SPEED_MM_S;

    // Outer velocity loop: positive speed error requests a forward lean.
    targetAngleDeg = velocityPid.update(targetSpeed - measuredSpeedMmS,
                                        velocityLoopDt);
  }

  // Inner balance loop: positive forward pitch commands positive forward effort.
  // Gyro rate is derivative-on-measurement for damping.
  const float balanceCommand = balancePid.update(
      pitchDeg - targetAngleDeg, dt, pitchRateDps, true);

  // Positive differential bias increases left-wheel speed: right turn.
  motors.setSpeeds(balanceCommand + nav.turnBiasPwm,
                   balanceCommand - nav.turnBiasPwm);
  printTelemetry(millis(), balanceCommand);
}
