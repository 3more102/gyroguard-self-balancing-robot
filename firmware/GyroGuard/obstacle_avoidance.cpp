#include "obstacle_avoidance.h"
#include "config.h"

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
  leftValid_ = false;
  rightValid_ = false;
  haveLeft_ = false;
  haveRight_ = false;
  frontDistanceMm_ = 0;
  frontDistanceValid_ = false;
  frontInvalidCount_ = 0;
  turnDirection_ = 0.0f;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  const bool wasEnabled = autonomous_;
  autonomous_ = enabled;
  if (!enabled) {
    state_ = DRIVE;
    servo_.write(SERVO_CENTER_DEG);
    frontInvalidCount_ = 0;
  } else if (!wasEnabled) {
    state_ = DRIVE;
    frontInvalidCount_ = 0;
    lastSonarStartMs_ = 0;
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
  leftValid_ = false;
  rightValid_ = false;
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

  if (state_ == DRIVE || state_ == SENSOR_HOLD) {
    frontDistanceValid_ = valid;
    if (valid) {
      frontDistanceMm_ = distance;
      frontInvalidCount_ = 0;
      if (autonomous_ && distance < OBSTACLE_STOP_MM) {
        startScan(nowMs);
      } else if (state_ == SENSOR_HOLD) {
        state_ = DRIVE;
        stateSinceMs_ = nowMs;
      }
    } else {
      if (frontInvalidCount_ < FRONT_SONAR_INVALID_LIMIT) {
        ++frontInvalidCount_;
      }
      if (autonomous_ &&
          frontInvalidCount_ >= FRONT_SONAR_INVALID_LIMIT) {
        state_ = SENSOR_HOLD;
        stateSinceMs_ = nowMs;
      }
    }
    return;
  }

  if (state_ == SCAN_LEFT_MEASURE) {
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

    if (!haveLeft_ || !haveRight_) {
      turnDirection_ = 0.0f;
      state_ = NO_PATH;
    } else {
      const RangeObservation left{leftDistanceMm_, leftValid_};
      const RangeObservation right{rightDistanceMm_, rightValid_};
      switch (selectAvoidanceAction(left, right, OBSTACLE_CLEAR_MM)) {
        case AvoidanceAction::TURN_LEFT:
          turnDirection_ = -1.0f;
          state_ = TURNING;
          break;
        case AvoidanceAction::TURN_RIGHT:
          turnDirection_ = 1.0f;
          state_ = TURNING;
          break;
        case AvoidanceAction::REVERSE:
          turnDirection_ = (leftDistanceMm_ >= rightDistanceMm_)
              ? -1.0f : 1.0f;
          state_ = REVERSING;
          break;
        case AvoidanceAction::HOLD:
          turnDirection_ = 0.0f;
          state_ = NO_PATH;
          break;
      }
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
      return {CRUISE_SPEED_MM_S, 0.0f};

    case SENSOR_HOLD:
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, SENSOR_HOLD);
      }
      return {0.0f, 0.0f};

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

    case NO_PATH:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= NO_PATH_RESCAN_MS) {
        startScan(nowMs);
      }
      return {0.0f, 0.0f};

    case TURNING:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= TURN_DURATION_MS) {
        state_ = DRIVE;
        stateSinceMs_ = nowMs;
        lastSonarStartMs_ = nowMs;
        return {CRUISE_SPEED_MM_S, 0.0f};
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
