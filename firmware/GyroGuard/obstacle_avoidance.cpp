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
  turnDirection_ = 0;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  autonomous_ = enabled;
  if (!enabled) {
    state_ = DRIVE;
    servo_.write(SERVO_CENTER_DEG);
    frontDistanceValid_ = false;
    leftValid_ = false;
    rightValid_ = false;
  }
}

bool ObstacleAvoidance::autonomous() const {
  return autonomous_;
}

bool ObstacleAvoidance::blocked() const {
  return state_ == BLOCKED;
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
    leftValid_ = valid;
    leftDistanceMm_ = valid ? distance : 0;
    servo_.write(SERVO_RIGHT_DEG);
    state_ = SCAN_RIGHT_SETTLE;
    stateSinceMs_ = nowMs;
  } else if (state_ == SCAN_RIGHT_MEASURE) {
    rightValid_ = valid;
    rightDistanceMm_ = valid ? distance : 0;

    turnDirection_ = gyroguard::chooseTurnDirection(
        {leftDistanceMm_, leftValid_},
        {rightDistanceMm_, rightValid_},
        OBSTACLE_CLEAR_MM);

    servo_.write(SERVO_CENTER_DEG);
    // If no direction has verified clearance, back away and then hold.
    state_ = (turnDirection_ == 0) ? REVERSING : TURNING;
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
      // Positive bias speeds the left wheel relative to the right: right turn.
      return {0.0f, static_cast<float>(turnDirection_) * TURN_BIAS_PWM};

    case REVERSING:
      if (static_cast<uint32_t>(nowMs - stateSinceMs_) >= REVERSE_DURATION_MS) {
        state_ = (turnDirection_ == 0) ? BLOCKED : TURNING;
        stateSinceMs_ = nowMs;
      }
      return {REVERSE_SPEED_MM_S, 0.0f};

    case BLOCKED:
      return {0.0f, 0.0f};
  }
  return {0.0f, 0.0f};
}
