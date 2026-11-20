#include <Arduino.h>
#include <avr/wdt.h>
#include "config.h"
#include "watchdog_startup.h"
#include "reset_policy.h"
#include "arm_guard.h"
#include "pid.h"
#include "imu_mpu6050.h"
#include "encoder.h"
#include "encoder_math.h"
#include "motion_guard.h"
#include "motor_driver.h"
#include "obstacle_avoidance.h"
#include "battery_monitor.h"
#include <math.h>

namespace {
MPU6050 imu;
EncoderPair encoders;
MotorDriver motors;
ObstacleAvoidance navigation;
BatteryMonitor batteryMonitor;

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
bool encoderFaultLatched = false;
bool encoderFaultMessagePending = false;
const __FlashStringHelper *pendingEncoderFaultReason = nullptr;
bool balanceArmed = false;
bool pitchValid = false;
uint32_t lastPitchSampleUs = 0;
bool telemetryEnabled = true;

uint32_t lastControlUs = 0;
uint32_t lastVelocityUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastBatteryTelemetryMs = 0;
float velocityLoopDt = 0.02f;
EncoderSnapshot previousCounts{0, 0};
MotionDemandMode previousMotionDemand = MOTION_DEMAND_NONE;
uint32_t lastLeftEncoderMotionUs = 0;
uint32_t lastRightEncoderMotionUs = 0;

void latchFault(const __FlashStringHelper *reason) {
  const bool firstFault = !faultLatched;
  faultLatched = true;
  balanceArmed = false;

  // Remove motor torque before navigation updates or serial diagnostics.
  motors.disable();
  gyroguard_clear_armed_marker();
  navigation.setAutonomous(false);

  if (firstFault) {
    Serial.print(F("FAULT: "));
    Serial.println(reason);
  }
}

void latchEncoderFault(const __FlashStringHelper *reason) {
  const bool firstFault = !encoderFaultLatched;
  encoderFaultLatched = true;
  navigation.setAutonomous(false);
  velocityPid.reset();
  targetAngleDeg = 0.0f;
  previousMotionDemand = MOTION_DEMAND_NONE;
  // Defer serial output until after the current balance command is applied.
  if (firstFault) {
    pendingEncoderFaultReason = reason;
    encoderFaultMessagePending = true;
  }
}

bool updateVelocity(uint32_t nowUs) {
  const uint32_t elapsedUs = nowUs - lastVelocityUs;
  if (elapsedUs < VELOCITY_PERIOD_US) return false;

  const EncoderSnapshot current = encoders.snapshot();
  const long leftDelta = current.left - previousCounts.left;
  const long rightDelta = current.right - previousCounts.right;
  if (leftDelta != 0) lastLeftEncoderMotionUs = nowUs;
  if (rightDelta != 0) lastRightEncoderMotionUs = nowUs;
  previousCounts = current;

  velocityLoopDt = static_cast<float>(elapsedUs) * 1.0e-6f;
  measuredSpeedMmS = encoderCountsToSpeedMmS(
      leftDelta, rightDelta, WHEEL_CIRCUMFERENCE_MM,
      ENCODER_COUNTS_PER_WHEEL_REV, velocityLoopDt);
  lastVelocityUs = nowUs;
  return true;
}

void printCommandResponse(const __FlashStringHelper *message,
                           int minimumFreeBytes) {
  if (Serial.availableForWrite() >= minimumFreeBytes) {
    Serial.println(message);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'b' || command == 'B') {
      const uint32_t commandNowUs = micros();
      if (!canArmBalance(faultLatched, pitchValid, pitchDeg, pitchRateDps,
                         ARM_TILT_LIMIT_DEG, ARM_RATE_LIMIT_DPS,
                         lastPitchSampleUs, commandNowUs,
                         MAX_ARM_SAMPLE_AGE_US)) {
        if (faultLatched) {
          printCommandResponse(F("ARM BLOCKED: fault"), 20);
        } else if (!pitchValid) {
          printCommandResponse(F("ARM BLOCKED: no IMU sample"), 30);
        } else if (!isPitchSampleFresh(lastPitchSampleUs, commandNowUs,
                                       MAX_ARM_SAMPLE_AGE_US)) {
          printCommandResponse(F("ARM BLOCKED: stale IMU sample"), 32);
        } else if (fabsf(pitchDeg) > ARM_TILT_LIMIT_DEG) {
          printCommandResponse(F("ARM BLOCKED: hold upright"), 28);
        } else {
          printCommandResponse(F("ARM BLOCKED: hold still"), 26);
        }
      } else if (!balanceArmed) {
        navigation.setAutonomous(false);
        balancePid.reset();
        velocityPid.reset();
        targetAngleDeg = 0.0f;
        measuredSpeedMmS = 0.0f;
        previousCounts = encoders.snapshot();
        lastVelocityUs = micros();
        gyroguard_mark_balance_armed();
        balanceArmed = true;
        printCommandResponse(F("BALANCE ARMED"), 16);
      } else {
        printCommandResponse(F("BALANCE ALREADY ARMED"), 24);
      }
    } else if (command == 'd' || command == 'D') {
      navigation.setAutonomous(false);
      balanceArmed = false;
      balancePid.reset();
      velocityPid.reset();
      targetAngleDeg = 0.0f;
      motors.disable();
      gyroguard_clear_armed_marker();
      printCommandResponse(F("DISARMED; support robot"), 28);
    } else if (command == 'a' || command == 'A') {
      if (faultLatched) {
        printCommandResponse(F("AUTO BLOCKED: fault"), 22);
      } else if (encoderFaultLatched) {
        printCommandResponse(F("AUTO BLOCKED: encoder fault; power-cycle"), 44);
      } else if (!balanceArmed) {
        printCommandResponse(F("ARM FIRST: send b"), 20);
      } else if (batteryMonitor.travelInhibited()) {
        printCommandResponse(F("AUTO BLOCKED: battery low/invalid"), 40);
      } else {
        navigation.setAutonomous(true);
        velocityPid.reset();
        printCommandResponse(F("AUTO=1"), 8);
      }
    } else if (command == 's' || command == 'S') {
      navigation.setAutonomous(false);
      velocityPid.reset();
      printCommandResponse(F("TRAVEL STOPPED"), 16);
    } else if (command == 't' || command == 'T') {
      telemetryEnabled = !telemetryEnabled;
    } else if (command == '?') {
      printCommandResponse(F("b=arm d=disarm a=auto s=stop t=telemetry"), 44);
    }
  }
}

void printTelemetry(uint32_t nowMs, float motorCommand) {
  if (!telemetryEnabled ||
      static_cast<uint32_t>(nowMs - lastTelemetryMs) < TELEMETRY_PERIOD_MS) return;

  // This compact record is at most 61 bytes including CR/LF. Do not block the
  // 200 Hz control loop while waiting for the UART transmit buffer.
  if (Serial.availableForWrite() < 63) return;
  lastTelemetryMs = nowMs;

  float speedDisplay = measuredSpeedMmS;
  float rateDisplay = pitchRateDps;
  if (!isfinite(speedDisplay)) speedDisplay = 0.0f;
  if (!isfinite(rateDisplay)) rateDisplay = 0.0f;
  if (speedDisplay > 9999.0f) speedDisplay = 9999.0f;
  if (speedDisplay < -9999.0f) speedDisplay = -9999.0f;
  if (rateDisplay > 999.0f) rateDisplay = 999.0f;
  if (rateDisplay < -999.0f) rateDisplay = -999.0f;

  Serial.print(F("p=")); Serial.print(pitchDeg, 1);
  Serial.print(F(",r=")); Serial.print(rateDisplay, 0);
  Serial.print(F(",v=")); Serial.print(speedDisplay, 0);
  Serial.print(F(",t=")); Serial.print(targetAngleDeg, 1);
  Serial.print(F(",m=")); Serial.print(motorCommand, 0);
  Serial.print(F(",d="));
  if (navigation.frontDistanceValid()) Serial.print(navigation.frontDistanceMm());
  else Serial.print(-1);
  Serial.print(F(",b=")); Serial.print(balanceArmed ? 1 : 0);
  Serial.print(F(",a=")); Serial.print(navigation.autonomous() ? 1 : 0);
  Serial.print(F(",e=")); Serial.print(encoderFaultLatched ? 1 : 0);
  Serial.print(F(",f=")); Serial.println(faultLatched ? 1 : 0);
}

void printBatteryTelemetry(uint32_t nowMs) {
  if (!telemetryEnabled ||
      static_cast<uint32_t>(nowMs - lastBatteryTelemetryMs) < 1000UL) return;
  // Keep the extra record short and non-blocking; the main control record
  // retains its existing bounded 200 Hz loop UART budget.
  if (Serial.availableForWrite() < 36) return;
  lastBatteryTelemetryMs = nowMs;
  Serial.print(F("batteryMv=")); Serial.print(batteryMonitor.millivolts());
  Serial.print(F(",valid=")); Serial.print(batteryMonitor.voltageValid() ? 1 : 0);
  Serial.print(F(",low=")); Serial.println(batteryMonitor.lowBattery() ? 1 : 0);
}
}

void setup() {
  // Put the H-bridge in standby before serial and sensor initialization.
  motors.begin();
  wdt_disable();
  Serial.begin(SERIAL_BAUD);
  batteryMonitor.begin();
  batteryMonitor.update(millis());
  const bool previousBalanceArmed = gyroguard_take_previous_armed_marker();
  const uint8_t resetCause = gyroguard_reset_cause;
  encoders.begin();
  navigation.begin();

  Serial.println(F("\nGyroGuard boot"));
  Serial.print(F("Previous reset flags: 0x"));
  Serial.println(resetCause, HEX);
  if (GyroGuardReset::requiresInspection(resetCause, previousBalanceArmed)) {
    if (previousBalanceArmed) {
      latchFault(F("Unexpected reset while balance was armed; inspect before re-arming"));
    } else if (GyroGuardReset::isBrownOutReset(resetCause)) {
      latchFault(F("Brown-out reset; inspect battery and wiring before re-arming"));
    } else if (GyroGuardReset::isApplicationWatchdogReset(resetCause)) {
      latchFault(F("Application watchdog reset; inspect robot before re-arming"));
    }
    return;
  }

#if defined(GYROGUARD_TEST_WATCHDOG_RESET)
  // Bench-only: verifies that the .noinit armed marker survives a watchdog reset.
  gyroguard_mark_balance_armed();
  wdt_enable(WDTO_15MS);
  while (true) {}
#endif
  Serial.println(F("Keep robot upright and motionless during IMU calibration."));

  if (!imu.begin()) {
    latchFault(F("MPU6050 not detected / I2C setup failed"));
    return;
  }

  if (!imu.calibrate(IMU_CALIBRATION_SAMPLES, imuZeroAngleDeg, gyroBiasDps)) {
    latchFault(F("IMU calibration unstable/failed; keep robot still"));
    return;
  }

  previousCounts = encoders.snapshot();
  lastControlUs = micros();
  lastVelocityUs = lastControlUs;
  lastLeftEncoderMotionUs = lastControlUs;
  lastRightEncoderMotionUs = lastControlUs;
  lastTelemetryMs = millis();
  lastBatteryTelemetryMs = millis();

  Serial.print(F("IMU zero angle: "));
  Serial.println(imuZeroAngleDeg, 3);
  Serial.print(F("Gyro Y bias dps: "));
  Serial.println(gyroBiasDps, 4);
  Serial.println(F("IMU calibrated. Motor outputs remain disabled."));
  Serial.println(F("Wait for valid pitch telemetry, hold within 8 degrees of upright and nearly motionless, then send 'b'."));
  Serial.println(F("Send 'a' only after balance-only operation is stable."));
  Serial.print(F("Battery monitor mV: "));
  Serial.println(batteryMonitor.millivolts());
  if (batteryMonitor.travelInhibited()) {
    Serial.println(F("BATTERY INHIBIT: autonomous travel requires valid voltage and recovery above 7.0 V."));
  }

  wdt_enable(WDTO_250MS);
}

void loop() {
  wdt_reset();
  const uint32_t batteryNowMs = millis();
  if (batteryMonitor.update(batteryNowMs) &&
      batteryMonitor.travelInhibited() && navigation.autonomous()) {
    navigation.setAutonomous(false);
    velocityPid.reset();
    targetAngleDeg = 0.0f;
    printCommandResponse(F("BATTERY INHIBIT: travel disabled; balance active"), 52);
  }
  readSerialCommands();

  if (faultLatched) {
    motors.disable();
    return;
  }

  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;

  const uint32_t elapsedUs = nowUs - lastControlUs;
  lastControlUs = nowUs;
  if (balanceArmed && elapsedUs > MAX_CONTROL_GAP_US) {
    latchFault(F("Control-loop timing overrun"));
    return;
  }

  const float dt = static_cast<float>(elapsedUs) * 1.0e-6f;
  if (!imu.readPitch(dt, imuZeroAngleDeg, gyroBiasDps, pitchDeg, pitchRateDps)) {
    latchFault(F("MPU6050 read failed"));
    return;
  }
  pitchValid = true;
  if (!isfinite(pitchDeg) || !isfinite(pitchRateDps)) {
    latchFault(F("Non-finite IMU estimate"));
    return;
  }
  lastPitchSampleUs = micros();

  if (balanceArmed && fabsf(pitchDeg) > MAX_SAFE_TILT_DEG) {
    latchFault(F("Robot exceeded safe tilt angle; power-cycle after inspection"));
    return;
  }

  const NavigationCommand nav = navigation.update(millis(), nowUs);
  const bool velocityUpdated = updateVelocity(nowUs);
  if (velocityUpdated && !isfinite(measuredSpeedMmS)) {
    latchFault(F("Non-finite encoder speed"));
    return;
  }

  if (!balanceArmed) {
    targetAngleDeg = 0.0f;
    motors.disable();
    printTelemetry(millis(), 0.0f);
    printBatteryTelemetry(millis());
    return;
  }

  const bool translationDemand =
      fabsf(nav.targetSpeedMmS) >= MIN_EXPECTED_TRAVEL_SPEED_MM_S;
  const bool rotationDemand =
      fabsf(nav.turnBiasPwm) >= MIN_EXPECTED_TURN_BIAS_PWM;
  MotionDemandMode demandMode = MOTION_DEMAND_NONE;
  if (translationDemand) demandMode = MOTION_DEMAND_TRANSLATION;
  else if (rotationDemand) demandMode = MOTION_DEMAND_ROTATION;

  // Start a fresh timeout window when autonomous motion begins or changes mode.
  if (demandMode != previousMotionDemand) {
    lastLeftEncoderMotionUs = nowUs;
    lastRightEncoderMotionUs = nowUs;
    previousMotionDemand = demandMode;
  }

  if (!encoderFaultLatched && navigation.autonomous() &&
      encoderFeedbackTimedOut(nowUs, lastLeftEncoderMotionUs,
                              lastRightEncoderMotionUs, demandMode,
                              ENCODER_STALL_TIMEOUT_US,
                              ENCODER_TURN_STALL_TIMEOUT_US)) {
    if (demandMode == MOTION_DEMAND_TRANSLATION) {
      latchEncoderFault(F("Missing wheel feedback during straight travel"));
    } else if (demandMode == MOTION_DEMAND_ROTATION) {
      latchEncoderFault(F("No wheel feedback during differential turn"));
    }
  }

  if (encoderFaultLatched) {
    targetAngleDeg = 0.0f;
  } else if (velocityUpdated) {
    float targetSpeed = nav.targetSpeedMmS;
    if (targetSpeed > MAX_TARGET_SPEED_MM_S) targetSpeed = MAX_TARGET_SPEED_MM_S;
    if (targetSpeed < -MAX_TARGET_SPEED_MM_S) targetSpeed = -MAX_TARGET_SPEED_MM_S;

    // Outer velocity loop: positive speed error requests a forward lean.
    targetAngleDeg = velocityPid.update(targetSpeed - measuredSpeedMmS,
                                        velocityLoopDt);
    if (!isfinite(targetAngleDeg)) {
      latchFault(F("Non-finite velocity controller output"));
      return;
    }
  }

  // Inner balance loop: positive forward pitch commands positive forward effort.
  // Gyro rate is derivative-on-measurement for damping.
  const float balanceCommand = balancePid.update(
      pitchDeg - targetAngleDeg, dt, pitchRateDps, true);
  if (!isfinite(balanceCommand)) {
    latchFault(F("Non-finite balance controller output"));
    return;
  }

  // Positive differential bias increases left-wheel speed: right turn.
  const float turnBias = encoderFaultLatched ? 0.0f : nav.turnBiasPwm;
  motors.setSpeeds(balanceCommand + turnBias,
                   balanceCommand - turnBias);
  if (encoderFaultMessagePending && Serial.availableForWrite() >= 63) {
    Serial.print(F("ENCODER FAULT: "));
    Serial.println(pendingEncoderFaultReason);
    encoderFaultMessagePending = false;
    pendingEncoderFaultReason = nullptr;
  }
  printTelemetry(millis(), balanceCommand);
  printBatteryTelemetry(millis());
}
