#include "obstacle_avoidance.h"
#include "config.h"
#include "navigation_logic.h"

void ObstacleAvoidance::begin() {
  servo_.attach(PIN_SERVO);
  servo_.write(SERVO_CENTER_DEG);
  sonar_.begin();
  state_ = DRIVE;
  autonomous_ = AUTO_DRIVE_ON_BOOT;
  stateSinceMs_ = millis();
  lastSonarStartMs_ = 0;
  leftDistanceMm_ = 0;
  rightDistanceMm_ = 0;
  frontDistanceMm_ = 0;
  frontDistanceValid_ = false;
  haveLeft_ = false;
  haveRight_ = false;
  leftValid_ = false;
  rightValid_ = false;
  turnDirection_ = 0.0f;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  if (!enabled) {
    autonomous_ = false;
    state_ = DRIVE;
    frontDistanceValid_ = false;
    servo_.write(SERVO_CENTER_DEG);
    return;
  }

  if (!autonomous_) {
    // Require a fresh, valid centre measurement before moving forward.
    state_ = DRIVE;
    frontDistanceValid_ = false;
    lastSonarStartMs_ = 0;
  }
  autonomous_ = true;
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
  leftValid_ = false;
  rightValid_ = false;
  frontDistanceValid_ = false;
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
    frontDistanceValid_ = valid;
    if (valid) frontDistanceMm_ = distance;

    if (valid && autonomous_ && distance < OBSTACLE_STOP_MM) {
      startScan(nowMs);
    }
  } else if (state_ == SCAN_LEFT_MEASURE) {
    leftDistanceMm_ = distance;
    leftValid_ = valid;
    haveLeft_ = true;
    servo_.write(SERVO_RIGHT_DEG);
    state_ = SCAN_RIGHT_SETTLE;
    stateSinceMs_ = nowMs;
  } else if (state_ == SCAN_RIGHT_MEASURE) {
    rightDistanceMm_ = distance;
    rightValid_ = valid;
    haveRight_ = true;
    servo_.write(SERVO_CENTER_DEG);

    if (haveLeft_ && haveRight_) {
      const ScanDecision decision = chooseScanDirection(
          leftValid_, leftDistanceMm_, rightValid_, rightDistanceMm_,
          OBSTACLE_CLEAR_MM);

      if (decision.hold) {
        // Never turn toward an unknown or insufficiently clear side.
        state_ = SENSOR_HOLD;
      } else {
        turnDirection_ = decision.turnDirection;
        state_ = TURNING;
      }
      stateSinceMs_ = nowMs;
    } else {
      state_ = SENSOR_HOLD;
      stateSinceMs_ = nowMs;
    }
  } else if (state_ == SENSOR_HOLD) {
    // Retry the centre sensor while stationary. Resume only after a valid
    // forward range proves that the stop-distance corridor is clear.
    frontDistanceValid_ = valid;
    if (valid) frontDistanceMm_ = distance;

    if (frontPathClear(valid, distance, OBSTACLE_STOP_MM)) {
      state_ = DRIVE;
      stateSinceMs_ = nowMs;
    } else if (valid) {
      startScan(nowMs);
    }
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
      // A missing, stale, too-close or out-of-range echo is not a clear path.
      return {frontPathClear(frontDistanceValid_, frontDistanceMm_,
                             OBSTACLE_STOP_MM)
                  ? CRUISE_SPEED_MM_S
                  : 0.0f,
              0.0f};

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
        frontDistanceValid_ = false;
        lastSonarStartMs_ = nowMs;
        return {0.0f, 0.0f};
      }
      // Positive bias speeds the left wheel relative to the right (right turn).
      return {0.0f, turnDirection_ * TURN_BIAS_PWM};

    case SENSOR_HOLD:
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, SENSOR_HOLD);
      }
      return {0.0f, 0.0f};
  }

  return {0.0f, 0.0f};
}
