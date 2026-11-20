#include "../firmware/GyroGuard/obstacle_avoidance.h"
#include "../firmware/GyroGuard/config.h"
#include <cassert>
#include <cmath>
#include <cstdint>

uint32_t fakeMillis = 0;
int fakeServoAngle = -1;

namespace {
bool sonarBusy = false;
bool sonarReady = false;
bool sonarInjected = false;
uint16_t sonarDistance = 0;
bool sonarValid = false;

void injectRange(uint16_t distanceMm, bool valid = true) {
  assert(sonarBusy);
  sonarDistance = distanceMm;
  sonarValid = valid;
  sonarInjected = true;
}

NavigationCommand step(ObstacleAvoidance &navigation, uint32_t nowMs) {
  fakeMillis = nowMs;
  return navigation.update(nowMs, nowMs * 1000UL);
}

bool near(float actual, float expected) {
  return std::fabs(actual - expected) < 0.001f;
}
}

// Deterministic host fake; production sonar.cpp remains AVR-specific.
void Ultrasonic::begin() {
  sonarBusy = false;
  sonarReady = false;
  sonarInjected = false;
  sonarDistance = 0;
  sonarValid = false;
}

void Ultrasonic::update(uint32_t) {
  if (sonarBusy && sonarInjected) {
    sonarBusy = false;
    sonarReady = true;
    sonarInjected = false;
  }
}

bool Ultrasonic::startMeasurement(uint32_t) {
  if (sonarBusy) return false;
  sonarBusy = true;
  return true;
}

bool Ultrasonic::takeDistanceMm(uint16_t &distanceMm, bool &valid) {
  if (!sonarReady) return false;
  distanceMm = sonarDistance;
  valid = sonarValid;
  sonarReady = false;
  return true;
}

bool Ultrasonic::busy() const { return sonarBusy; }

static void testRearmWaitsForSettledCenterAndValidRange() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  NavigationCommand command = step(navigation, SERVO_SETTLE_MS - 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!sonarBusy);

  command = step(navigation, SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(0, false);
  command = step(navigation, SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  command = step(navigation, 2 * SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(900, true);
  command = step(navigation, 2 * SERVO_SETTLE_MS + 2);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
  assert(navigation.frontDistanceValid());
}

static void testTurnRequiresFreshSettledCenterReading() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  step(navigation, SERVO_SETTLE_MS);
  injectRange(250, true);
  NavigationCommand command = step(navigation, SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(fakeServoAngle == SERVO_LEFT_DEG);

  step(navigation, SERVO_SETTLE_MS + 1 + SERVO_SETTLE_MS);
  injectRange(800, true);
  step(navigation, SERVO_SETTLE_MS + 2 + SERVO_SETTLE_MS);
  assert(fakeServoAngle == SERVO_RIGHT_DEG);

  step(navigation, SERVO_SETTLE_MS + 2 + 2 * SERVO_SETTLE_MS);
  injectRange(300, true);
  command = step(navigation, SERVO_SETTLE_MS + 3 + 2 * SERVO_SETTLE_MS);
  assert(fakeServoAngle == SERVO_CENTER_DEG);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(command.turnBiasPwm < 0.0f); // The left side has verified clearance.

  const uint32_t turnFinished =
      SERVO_SETTLE_MS + 3 + 2 * SERVO_SETTLE_MS + TURN_DURATION_MS;
  command = step(navigation, turnFinished);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  command = step(navigation, turnFinished + SERVO_SETTLE_MS - 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!sonarBusy);

  command = step(navigation, turnFinished + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(900, true);
  command = step(navigation, turnFinished + SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testBlockedSidesHoldAndCenterClearanceBelowThresholdRescans() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  step(navigation, SERVO_SETTLE_MS);
  injectRange(250, true);
  step(navigation, SERVO_SETTLE_MS + 1);

  step(navigation, SERVO_SETTLE_MS + 1 + SERVO_SETTLE_MS);
  injectRange(200, true);
  step(navigation, SERVO_SETTLE_MS + 2 + SERVO_SETTLE_MS);

  step(navigation, SERVO_SETTLE_MS + 2 + 2 * SERVO_SETTLE_MS);
  injectRange(350, true);
  NavigationCommand command = step(
      navigation, SERVO_SETTLE_MS + 3 + 2 * SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(near(command.turnBiasPwm, 0.0f));
  assert(fakeServoAngle == SERVO_CENTER_DEG);

  const uint32_t holdStarted =
      SERVO_SETTLE_MS + 3 + 2 * SERVO_SETTLE_MS;
  command = step(navigation, holdStarted + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  // 400 mm is above the stop threshold but below the clear threshold.
  injectRange(400, true);
  command = step(navigation, holdStarted + SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(fakeServoAngle == SERVO_LEFT_DEG);
}

static void testSideEchoIsDiscardedWhenAutonomyIsDisabled() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  step(navigation, SERVO_SETTLE_MS);
  injectRange(250, true);
  step(navigation, SERVO_SETTLE_MS + 1);

  step(navigation, SERVO_SETTLE_MS + 1 + SERVO_SETTLE_MS);
  assert(sonarBusy);
  navigation.setAutonomous(false);
  injectRange(800, true);
  NavigationCommand command = step(
      navigation, SERVO_SETTLE_MS + 2 + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(fakeServoAngle == SERVO_CENTER_DEG);

  navigation.setAutonomous(true);
  const uint32_t rearmAt = SERVO_SETTLE_MS + 2 + SERVO_SETTLE_MS;
  command = step(navigation, rearmAt + SERVO_SETTLE_MS - 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!sonarBusy);

  command = step(navigation, rearmAt + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(900, true);
  command = step(navigation, rearmAt + SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}


static void testSideEchoArrivingAfterRearmIsDiscarded() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  step(navigation, SERVO_SETTLE_MS);
  injectRange(250, true);
  step(navigation, SERVO_SETTLE_MS + 1);

  const uint32_t sideMeasurementStarted =
      SERVO_SETTLE_MS + 1 + SERVO_SETTLE_MS;
  step(navigation, sideMeasurementStarted);
  assert(sonarBusy); // The left-side echo is pending.

  navigation.setAutonomous(false);
  navigation.setAutonomous(true); // Re-arm before that side echo arrives.

  injectRange(800, true); // A side result must not be accepted as a center result.
  NavigationCommand command = step(navigation, sideMeasurementStarted + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(!sonarBusy); // The controller is waiting for center-servo settling.

  command = step(
      navigation, sideMeasurementStarted + SERVO_SETTLE_MS - 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!sonarBusy);

  command = step(
      navigation, sideMeasurementStarted + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy); // Only now may a new center measurement begin.

  injectRange(900, true);
  command = step(
      navigation, sideMeasurementStarted + SERVO_SETTLE_MS + 1);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 900);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testStaleForwardRangeStopsCruise() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  step(navigation, SERVO_SETTLE_MS);
  injectRange(900, true);
  NavigationCommand command = step(navigation, SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));

  command = step(navigation, SERVO_SETTLE_MS + FRONT_RANGE_MAX_AGE_MS);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));

  command = step(
      navigation, SERVO_SETTLE_MS + FRONT_RANGE_MAX_AGE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
}

int main() {
  testRearmWaitsForSettledCenterAndValidRange();
  testTurnRequiresFreshSettledCenterReading();
  testBlockedSidesHoldAndCenterClearanceBelowThresholdRescans();
  testSideEchoIsDiscardedWhenAutonomyIsDisabled();
  testSideEchoArrivingAfterRearmIsDiscarded();
  testStaleForwardRangeStopsCruise();
  return 0;
}
