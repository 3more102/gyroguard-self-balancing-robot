#include "obstacle_avoidance.h"
#include "navigation_logic.h"
#include "config.h"

void ObstacleAvoidance::begin() {
  servo_.attach(PIN_SERVO);
  servo_.write(SERVO_CENTER_DEG);
  sonar_.begin();
  autonomous_ = AUTO_DRIVE_ON_BOOT;
  state_ = autonomous_ ? WAIT_FRONT : DRIVE;
  ignoreNextSonarResult_ = false;
  stateSinceMs_ = millis();
  lastSonarStartMs_ = 0;
  leftValid_ = false;
  rightValid_ = false;
  frontDistanceMm_ = 0;
  frontDistanceValid_ = false;
  turnDirection_ = 0.0f;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  if (sonar_.busy()) ignoreNextSonarResult_ = true;

  autonomous_ = enabled;
  frontDistanceValid_ = false;
  servo_.write(SERVO_CENTER_DEG);

  if (enabled) {
    state_ = WAIT_FRONT;
    lastSonarStartMs_ = millis();
  } else {
    state_ = DRIVE;
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

  // A measurement started before an autonomy-mode change may have been taken
  // with the servo pointed sideways. Never reinterpret it as a front reading.
  if (ignoreNextSonarResult_) {
    ignoreNextSonarResult_ = false;
    return;
  }

  if (state_ == DRIVE || state_ == WAIT_FRONT_MEASURE) {
    const bool wasWaitingForFront = state_ == WAIT_FRONT_MEASURE;
    frontDistanceValid_ = valid;
    if (valid) frontDistanceMm_ = distance;

    if (!autonomous_) return;

    // Unknown front range is not evidence of a clear path. Hold travel while
    // preserving the independent balance controller, then retry ranging.
    if (!valid) {
      state_ = WAIT_FRONT;
      stateSinceMs_ = nowMs;
      return;
    }

    if (distance < OBSTACLE_STOP_MM) {
      startScan(nowMs);
    } else if (wasWaitingForFront) {
      state_ = DRIVE;
      stateSinceMs_ = nowMs;
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

    const ScanDecision decision = decideScan(
        leftValid_, leftDistanceMm_, rightValid_, rightDistanceMm_,
        OBSTACLE_CLEAR_MM);

    servo_.write(SERVO_CENTER_DEG);
    turnDirection_ = static_cast<float>(decision.turnDirection);

    if (decision.action == ScanAction::Hold) {
      // Both side readings are unknown. Do not reverse or turn blindly.
      state_ = BLOCKED;
    } else if (decision.action == ScanAction::Reverse) {
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
    case WAIT_FRONT:
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, WAIT_FRONT_MEASURE);
      }
      return {0.0f, 0.0f};

    case WAIT_FRONT_MEASURE:
      return {0.0f, 0.0f};

    case DRIVE:
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, DRIVE);
      }
      return {CRUISE_SPEED_MM_S, 0.0f};

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
        state_ = WAIT_FRONT;
        stateSinceMs_ = nowMs;
        lastSonarStartMs_ = nowMs;
        frontDistanceValid_ = false;
        return {0.0f, 0.0f};
      }
      // Positive bias speeds the left wheel relative to the right wheel.
      return {0.0f, turnDirection_ * TURN_BIAS_PWM};

    case REVERSING:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= REVERSE_DURATION_MS) {
        state_ = TURNING;
        stateSinceMs_ = nowMs;
      }
      return {REVERSE_SPEED_MM_S, 0.0f};

    case BLOCKED:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= BLOCKED_RESCAN_MS) {
        startScan(nowMs);
      }
      return {0.0f, 0.0f};
  }

  return {0.0f, 0.0f};
}
