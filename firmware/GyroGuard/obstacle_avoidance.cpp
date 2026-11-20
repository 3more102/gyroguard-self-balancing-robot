#include "obstacle_avoidance.h"
#include "config.h"
#include "navigation_policy.h"

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
  leftValid_ = false;
  rightValid_ = false;
  ignoreNextSonar_ = false;
  turnDirection_ = 0.0f;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  autonomous_ = enabled;
  state_ = DRIVE;
  servo_.write(SERVO_CENTER_DEG);
  frontDistanceValid_ = false;

  if (enabled) {
    // Discard a completed reading from the previous scan/mode. If an echo is
    // still in flight, ignore its completion before accepting a front sample.
    uint16_t discardedDistanceMm = 0;
    bool discardedValid = false;
    (void)sonar_.takeDistanceMm(discardedDistanceMm, discardedValid);
    ignoreNextSonar_ = sonar_.busy();
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
  leftDistanceMm_ = 0;
  rightDistanceMm_ = 0;
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

  if (ignoreNextSonar_) {
    ignoreNextSonar_ = false;
    return;
  }

  if (state_ == DRIVE) {
    frontDistanceValid_ = valid;
    if (valid) {
      frontDistanceMm_ = distance;
      if (autonomous_ && distance < OBSTACLE_STOP_MM) {
        startScan(nowMs);
      }
    }
  } else if (state_ == SCAN_LEFT_MEASURE) {
    leftValid_ = valid;
    leftDistanceMm_ = valid ? distance : 0;
    servo_.write(SERVO_RIGHT_DEG);
    state_ = SCAN_RIGHT_SETTLE;
    stateSinceMs_ = nowMs;
  } else if (state_ == SCAN_RIGHT_MEASURE) {
    rightValid_ = valid;
    rightDistanceMm_ = valid ? distance : 0;

    const GyroGuard::TurnDecision decision =
        GyroGuard::chooseTurnDirection(leftValid_, leftDistanceMm_,
                                       rightValid_, rightDistanceMm_,
                                       OBSTACLE_CLEAR_MM);

    servo_.write(SERVO_CENTER_DEG);
    if (decision == GyroGuard::TurnDecision::HOLD) {
      // Neither side has proven clearance. Do not guess and do not reverse
      // blindly; settle at center and obtain a fresh forward measurement.
      frontDistanceValid_ = false;
      state_ = CENTER_SETTLE;
    } else {
      turnDirection_ = (decision == GyroGuard::TurnDecision::LEFT)
                           ? -1.0f
                           : 1.0f;
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
      // Fail closed: no valid front echo means no autonomous forward motion.
      if (GyroGuard::forwardPathIsClear(frontDistanceValid_, frontDistanceMm_,
                                        OBSTACLE_STOP_MM)) {
        return {CRUISE_SPEED_MM_S, 0.0f};
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

    case CENTER_SETTLE:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= SERVO_SETTLE_MS) {
        state_ = DRIVE;
        lastSonarStartMs_ = nowMs;
      }
      return {0.0f, 0.0f};

    case TURNING:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= TURN_DURATION_MS) {
        state_ = DRIVE;
        stateSinceMs_ = nowMs;
        frontDistanceValid_ = false;
        lastSonarStartMs_ = nowMs;
        return {0.0f, 0.0f};
      }
      // Positive bias speeds left wheel relative to right wheel (right turn).
      return {0.0f, turnDirection_ * TURN_BIAS_PWM};
  }

  return {0.0f, 0.0f};
}
