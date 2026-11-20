#include "obstacle_avoidance.h"
#include "config.h"

void ObstacleAvoidance::begin() {
  servo_.attach(PIN_SERVO);
  servo_.write(SERVO_CENTER_DEG);
  sonar_.begin();
  state_ = DRIVE;
  autonomous_ = AUTO_DRIVE_ON_BOOT;
  discardNextSonar_ = false;
  stateSinceMs_ = millis();
  lastSonarStartMs_ = 0;
  leftDistanceMm_ = 0;
  rightDistanceMm_ = 0;
  frontDistanceMm_ = 0;
  leftValid_ = false;
  rightValid_ = false;
  frontDistanceValid_ = false;
  turnDirection_ = 0.0f;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  if (autonomous_ == enabled) return;

  autonomous_ = enabled;
  const uint32_t nowMs = millis();

  if (!enabled) {
    // A measurement taken while the scanner is pointed sideways must not be
    // reused later as a forward range measurement.
    if (state_ == SCAN_LEFT_MEASURE || state_ == SCAN_RIGHT_MEASURE) {
      discardNextSonar_ = sonar_.busy();
      if (discardNextSonar_) frontDistanceValid_ = false;
    }
    state_ = DRIVE;
    servo_.write(SERVO_CENTER_DEG);
    stateSinceMs_ = nowMs;
    lastSonarStartMs_ = nowMs;
    return;
  }

  servo_.write(SERVO_CENTER_DEG);
  stateSinceMs_ = nowMs;
  lastSonarStartMs_ = nowMs;
  state_ = (frontDistanceValid_ && frontDistanceMm_ >= OBSTACLE_CLEAR_MM)
      ? DRIVE : HOLD;
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

  if (discardNextSonar_) {
    discardNextSonar_ = false;
    return;
  }

  if (state_ == DRIVE) {
    frontDistanceValid_ = valid;
    if (valid) frontDistanceMm_ = distance;

    if (autonomous_ && !valid) {
      // Unknown forward space is not treated as clear.
      state_ = HOLD;
      stateSinceMs_ = nowMs;
      lastSonarStartMs_ = nowMs;
      servo_.write(SERVO_CENTER_DEG);
    } else if (valid && autonomous_ && distance < OBSTACLE_STOP_MM) {
      startScan(nowMs);
    }
  } else if (state_ == HOLD) {
    frontDistanceValid_ = valid;
    if (valid) frontDistanceMm_ = distance;

    if (valid && distance >= OBSTACLE_CLEAR_MM) {
      state_ = DRIVE;
      stateSinceMs_ = nowMs;
      lastSonarStartMs_ = nowMs;
    }
  } else if (state_ == SCAN_LEFT_MEASURE) {
    leftDistanceMm_ = distance;
    leftValid_ = valid;
    servo_.write(SERVO_RIGHT_DEG);
    state_ = SCAN_RIGHT_SETTLE;
    stateSinceMs_ = nowMs;
  } else if (state_ == SCAN_RIGHT_MEASURE) {
    rightDistanceMm_ = distance;
    rightValid_ = valid;

    const AvoidanceTurn choice = chooseAvoidanceTurn(
        leftValid_, leftDistanceMm_, rightValid_, rightDistanceMm_,
        static_cast<uint16_t>(OBSTACLE_CLEAR_MM));

    servo_.write(SERVO_CENTER_DEG);
    if (choice == AvoidanceTurn::HOLD) {
      // There is no rear-facing sensor: never reverse blindly.
      state_ = HOLD;
      stateSinceMs_ = nowMs;
      lastSonarStartMs_ = nowMs;
    } else {
      turnDirection_ = (choice == AvoidanceTurn::LEFT) ? -1.0f : 1.0f;
      state_ = TURNING;
      stateSinceMs_ = nowMs;
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
      return {CRUISE_SPEED_MM_S, 0.0f};

    case HOLD:
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - stateSinceMs_) >= HOLD_RESCAN_MS) {
        startScan(nowMs);
        return {0.0f, 0.0f};
      }
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - stateSinceMs_) >= SERVO_SETTLE_MS &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, HOLD);
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

    case TURNING:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= TURN_DURATION_MS) {
        state_ = DRIVE;
        stateSinceMs_ = nowMs;
        lastSonarStartMs_ = nowMs;
        return {CRUISE_SPEED_MM_S, 0.0f};
      }
      // Positive bias speeds the left wheel relative to the right wheel.
      return {0.0f, turnDirection_ * TURN_BIAS_PWM};
  }

  return {0.0f, 0.0f};
}
