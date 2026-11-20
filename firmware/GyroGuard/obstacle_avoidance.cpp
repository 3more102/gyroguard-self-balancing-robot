#include "obstacle_avoidance.h"
#include "config.h"
#include "navigation_policy.h"
#include "range_guard.h"

void ObstacleAvoidance::begin() {
  servo_.attach(PIN_SERVO);
  servo_.write(SERVO_CENTER_DEG);
  sonar_.begin();
  state_ = DRIVE;
  autonomous_ = AUTO_DRIVE_ON_BOOT;
  stateSinceMs_ = millis();
  lastSonarStartMs_ = 0;
  frontDistanceAtMs_ = 0;
  leftDistanceMm_ = 0;
  rightDistanceMm_ = 0;
  leftValid_ = false;
  rightValid_ = false;
  frontDistanceMm_ = 0;
  frontDistanceValid_ = false;
  ignoreNextSonarResult_ = false;
  turnDirection_ = 0.0f;
}

void ObstacleAvoidance::setAutonomous(bool enabled) {
  if (enabled == autonomous_) return;

  if (!enabled) {
    // A side-scan echo must not be mistaken for a forward-facing reading
    // after the servo is returned to its center position.
    ignoreNextSonarResult_ =
        state_ == SCAN_LEFT_MEASURE ||
        state_ == SCAN_RIGHT_MEASURE ||
        state_ == SENSOR_HOLD_MEASURE;
    autonomous_ = false;
    state_ = DRIVE;
    servo_.write(SERVO_CENTER_DEG);
  } else {
    autonomous_ = true;
    state_ = DRIVE;
    servo_.write(SERVO_CENTER_DEG);
    lastSonarStartMs_ = 0;
  }

  frontDistanceMm_ = 0;
  frontDistanceValid_ = false;
  frontDistanceAtMs_ = 0;
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
  frontDistanceMm_ = 0;
  frontDistanceValid_ = false;
  frontDistanceAtMs_ = 0;
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

  if (ignoreNextSonarResult_) {
    ignoreNextSonarResult_ = false;
    return;
  }

  if (state_ == DRIVE) {
    frontDistanceValid_ = valid;
    frontDistanceMm_ = valid ? distance : 0;
    frontDistanceAtMs_ = valid ? nowMs : 0;

    if (autonomous_) {
      if (!valid) {
        // No forward motion is permitted until the center sensor recovers.
        state_ = SENSOR_HOLD;
        stateSinceMs_ = nowMs;
        servo_.write(SERVO_CENTER_DEG);
      } else if (distance < OBSTACLE_STOP_MM) {
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

    const NavigationDecision decision = chooseNavigationDecision(
        leftValid_, leftDistanceMm_, rightValid_, rightDistanceMm_,
        static_cast<uint16_t>(OBSTACLE_CLEAR_MM));

    servo_.write(SERVO_CENTER_DEG);
    if (decision.valid) {
      turnDirection_ = static_cast<float>(decision.turnDirection);
      state_ = TURNING;
    } else {
      // Neither side has enough verified clearance. Hold position and retry.
      state_ = SENSOR_HOLD;
      frontDistanceMm_ = 0;
      frontDistanceValid_ = false;
      frontDistanceAtMs_ = 0;
    }
    stateSinceMs_ = nowMs;
  } else if (state_ == SENSOR_HOLD_MEASURE) {
    frontDistanceValid_ = valid;
    frontDistanceMm_ = valid ? distance : 0;
    frontDistanceAtMs_ = valid ? nowMs : 0;

    if (!valid) {
      state_ = SENSOR_HOLD;
      stateSinceMs_ = nowMs;
    } else if (distance < OBSTACLE_STOP_MM) {
      startScan(nowMs);
    } else {
      state_ = DRIVE;
      stateSinceMs_ = nowMs;
    }
  }
}

NavigationCommand ObstacleAvoidance::update(uint32_t nowMs, uint32_t nowUs) {
  sonar_.update(nowUs);
  consumeSonar(nowMs);

  if (!autonomous_) return {0.0f, 0.0f};

  switch (state_) {
    case DRIVE: {
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, DRIVE);
      }
      const bool rangeFresh = isRangeSampleFresh(
          frontDistanceValid_, frontDistanceAtMs_, nowMs,
          FRONT_RANGE_MAX_AGE_MS);
      return {rangeFresh ? CRUISE_SPEED_MM_S : 0.0f, 0.0f};
    }

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
        frontDistanceMm_ = 0;
        frontDistanceValid_ = false;
        frontDistanceAtMs_ = 0;
        // Wait for a fresh center reading before driving forward again.
        return {0.0f, 0.0f};
      }
      // Positive bias speeds the left wheel relative to the right: turn right.
      return {0.0f, turnDirection_ * TURN_BIAS_PWM};

    case SENSOR_HOLD:
      if (!sonar_.busy() &&
          static_cast<uint32_t>(nowMs - lastSonarStartMs_) >= SONAR_RETRY_MS) {
        startSonar(nowUs, nowMs, SENSOR_HOLD_MEASURE);
      }
      return {0.0f, 0.0f};

    case SENSOR_HOLD_MEASURE:
      return {0.0f, 0.0f};
  }

  return {0.0f, 0.0f};
}
