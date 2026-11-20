#include "obstacle_avoidance.h"
#include "config.h"
#include "navigation_safety.h"

void ObstacleAvoidance::begin() {
  servo_.attach(PIN_SERVO);
  servo_.write(SERVO_CENTER_DEG);
  sonar_.begin();
  state_ = DRIVE;
  autonomous_ = AUTO_DRIVE_ON_BOOT;
  stateSinceMs_ = millis();
  lastSonarStartMs_ = 0;
  haveLeft_ = false;
  haveRight_ = false;
  frontDistanceMm_ = 0;
  frontDistanceValid_ = false;
  lastFrontMeasurementMs_ = 0;
  turnDirection_ = 0.0f;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  autonomous_ = enabled;
  if (!enabled) {
    state_ = DRIVE;
    servo_.write(SERVO_CENTER_DEG);
  }
}

bool ObstacleAvoidance::autonomous() const {
  return autonomous_;
}

bool ObstacleAvoidance::frontDistanceValid() const {
  return frontDistanceValid_;
}

uint16_t ObstacleAvoidance::frontDistanceMm() const {
  return frontDistanceMm_;
}

void ObstacleAvoidance::startScan(uint32_t nowMs) {
  haveLeft_ = false;
  haveRight_ = false;
  servo_.write(SERVO_LEFT_DEG);
  state_ = SCAN_LEFT_SETTLE;
  stateSinceMs_ = nowMs;
}

void ObstacleAvoidance::startSonar(uint32_t nowUs, uint32_t nowMs,
                                   State measureState) {
  if (sonar_.startMeasurement(nowUs)) {
    state_ = measureState;
    stateSinceMs_ = nowMs;
    lastSonarStartMs_ = nowMs;
  }
}

void ObstacleAvoidance::consumeSonar(uint32_t nowMs) {
  uint16_t distance = 0;
  bool valid = false;
  if (!sonar_.takeDistanceMm(distance, valid)) return;

  if (state_ == DRIVE) {
    lastFrontMeasurementMs_ = nowMs;
    frontDistanceValid_ = valid;
    if (valid) frontDistanceMm_ = distance;
    if (valid && autonomous_ && distance < OBSTACLE_STOP_MM) {
      startScan(nowMs);
    }
  } else if (state_ == SCAN_LEFT_MEASURE) {
    leftDistanceMm_ = gyroguardConservativeScanDistance(valid, distance);
    haveLeft_ = true;
    servo_.write(SERVO_RIGHT_DEG);
    state_ = SCAN_RIGHT_SETTLE;
    stateSinceMs_ = nowMs;
  } else if (state_ == SCAN_RIGHT_MEASURE) {
    rightDistanceMm_ = gyroguardConservativeScanDistance(valid, distance);
    haveRight_ = true;

    if (haveLeft_ && haveRight_) {
      turnDirection_ = (leftDistanceMm_ > rightDistanceMm_) ? -1.0f : 1.0f;
    } else if (haveLeft_) {
      turnDirection_ = -1.0f;
    } else {
      turnDirection_ = 1.0f;
    }

    servo_.write(SERVO_CENTER_DEG);
    if (leftDistanceMm_ < OBSTACLE_CLEAR_MM &&
        rightDistanceMm_ < OBSTACLE_CLEAR_MM) {
      state_ = REVERSING;
    } else {
      state_ = TURNING;
    }
    stateSinceMs_ = nowMs;
  }
}

NavigationCommand ObstacleAvoidance::update(uint32_t nowMs, uint32_t nowUs) {
  sonar_.update(nowUs);
  consumeSonar(nowMs);

  if (!autonomous_) return {0.0f, 0.0f};

  switch (state_) {
    case DRIVE:
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, DRIVE);
      }
      const bool rangeSafe = gyroguardFrontRangeAllowsCruise(
          frontDistanceValid_, frontDistanceMm_, nowMs, lastFrontMeasurementMs_,
          SONAR_DATA_TIMEOUT_MS, static_cast<uint16_t>(OBSTACLE_STOP_MM));
      return {rangeSafe ? CRUISE_SPEED_MM_S : 0.0f, 0.0f};

    case SCAN_LEFT_SETTLE:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= SERVO_SETTLE_MS) {
        startSonar(nowUs, nowMs, SCAN_LEFT_MEASURE);
      }
      return {0.0f, 0.0f};

    case SCAN_LEFT_MEASURE:
      return {0.0f, 0.0f};

    case SCAN_RIGHT_SETTLE:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= SERVO_SETTLE_MS) {
        startSonar(nowUs, nowMs, SCAN_RIGHT_MEASURE);
      }
      return {0.0f, 0.0f};

    case SCAN_RIGHT_MEASURE:
      return {0.0f, 0.0f};

    case TURNING:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= TURN_DURATION_MS) {
        state_ = DRIVE;
        stateSinceMs_ = nowMs;
        lastSonarStartMs_ = nowMs;
        return {0.0f, 0.0f};
      }
      // Positive bias speeds left wheel relative to right wheel (right turn).
      return {0.0f, turnDirection_ * TURN_BIAS_PWM};

    case REVERSING:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= REVERSE_DURATION_MS) {
        state_ = TURNING;
        stateSinceMs_ = nowMs;
      }
      return {REVERSE_SPEED_MM_S, 0.0f};
  }
  return {0.0f, 0.0f};
}
