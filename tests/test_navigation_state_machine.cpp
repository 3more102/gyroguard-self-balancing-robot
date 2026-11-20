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

NavigationCommand updateAt(ObstacleAvoidance &navigation, uint32_t nowMs) {
  fakeMillis = nowMs;
  return navigation.update(nowMs, nowMs * 1000UL);
}

bool near(float actual, float expected) {
  return std::fabs(actual - expected) < 0.001f;
}

uint32_t armAndStartCenterMeasurement(ObstacleAvoidance &navigation) {
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  NavigationCommand command = updateAt(navigation, SERVO_SETTLE_MS - 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!sonarBusy);

  command = updateAt(navigation, SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);
  return SERVO_SETTLE_MS;
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

static void testInvalidFrontRangeHoldsUntilSensorRecovers() {
  ObstacleAvoidance navigation;
  const uint32_t frontStart = armAndStartCenterMeasurement(navigation);

  injectRange(0, false);
  NavigationCommand command = updateAt(navigation, frontStart + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  // An invalid center sample enters hold; the centered servo must settle again.
  const uint32_t retryAt = frontStart + 1 + SERVO_SETTLE_MS;
  command = updateAt(navigation, retryAt);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(900, true);
  command = updateAt(navigation, retryAt + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 900);
}

static void testTurnRequiresFreshFrontRangeAfterward() {
  ObstacleAvoidance navigation;
  const uint32_t frontStart = armAndStartCenterMeasurement(navigation);

  injectRange(250, true);
  NavigationCommand command = updateAt(navigation, frontStart + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(fakeServoAngle == SERVO_LEFT_DEG);

  const uint32_t leftStart = frontStart + 1 + SERVO_SETTLE_MS;
  updateAt(navigation, leftStart);
  injectRange(800, true);
  updateAt(navigation, leftStart + 1);
  assert(fakeServoAngle == SERVO_RIGHT_DEG);

  const uint32_t rightStart = leftStart + 1 + SERVO_SETTLE_MS;
  updateAt(navigation, rightStart);
  injectRange(300, true);
  command = updateAt(navigation, rightStart + 1);
  assert(fakeServoAngle == SERVO_CENTER_DEG);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(command.turnBiasPwm < 0.0f); // The left side has verified clearance.

  const uint32_t turnFinished = rightStart + 1 + TURN_DURATION_MS;
  command = updateAt(navigation, turnFinished);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  const uint32_t centerStart = turnFinished + SERVO_SETTLE_MS;
  command = updateAt(navigation, centerStart);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);
  injectRange(900, true);
  command = updateAt(navigation, centerStart + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testBothBlockedSidesHoldAndRescan() {
  ObstacleAvoidance navigation;
  const uint32_t frontStart = armAndStartCenterMeasurement(navigation);

  injectRange(250, true);
  updateAt(navigation, frontStart + 1);

  const uint32_t leftStart = frontStart + 1 + SERVO_SETTLE_MS;
  updateAt(navigation, leftStart);
  injectRange(200, true);
  updateAt(navigation, leftStart + 1);

  const uint32_t rightStart = leftStart + 1 + SERVO_SETTLE_MS;
  updateAt(navigation, rightStart);
  injectRange(350, true);
  NavigationCommand command = updateAt(navigation, rightStart + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(near(command.turnBiasPwm, 0.0f));
  assert(fakeServoAngle == SERVO_CENTER_DEG);

  // No rear sensor exists: the robot holds instead of reversing blindly.
  const uint32_t centerStart = rightStart + 1 + SERVO_SETTLE_MS;
  command = updateAt(navigation, centerStart);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(250, true);
  command = updateAt(navigation, centerStart + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(fakeServoAngle == SERVO_LEFT_DEG); // Still blocked: rescan, don't drive.
}

static void testSideEchoIsDiscardedWhenAutonomyIsDisabled() {
  ObstacleAvoidance navigation;
  const uint32_t frontStart = armAndStartCenterMeasurement(navigation);

  injectRange(250, true);
  updateAt(navigation, frontStart + 1);

  const uint32_t leftStart = frontStart + 1 + SERVO_SETTLE_MS;
  updateAt(navigation, leftStart);
  assert(sonarBusy);
  navigation.setAutonomous(false);
  injectRange(800, true);
  NavigationCommand command = updateAt(navigation, leftStart + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(fakeServoAngle == SERVO_CENTER_DEG);

  navigation.setAutonomous(true);
  const uint32_t centerStart = leftStart + 1 + SERVO_SETTLE_MS;
  command = updateAt(navigation, centerStart);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);
  injectRange(900, true);
  command = updateAt(navigation, centerStart + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testSideEchoArrivingAfterRearmIsDiscarded() {
  ObstacleAvoidance navigation;
  const uint32_t frontStart = armAndStartCenterMeasurement(navigation);

  injectRange(250, true);
  updateAt(navigation, frontStart + 1);

  const uint32_t leftStart = frontStart + 1 + SERVO_SETTLE_MS;
  updateAt(navigation, leftStart);
  assert(sonarBusy); // Left-side measurement is in flight.

  navigation.setAutonomous(false);
  navigation.setAutonomous(true); // Re-arm before the old echo arrives.

  injectRange(800, true); // This is still the old left-side result.
  NavigationCommand command = updateAt(navigation, leftStart + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(!sonarBusy); // The center servo must settle before a new ping.

  const uint32_t centerStart = leftStart + SERVO_SETTLE_MS;
  command = updateAt(navigation, centerStart - 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!sonarBusy);

  command = updateAt(navigation, centerStart);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy); // Only a new center measurement may authorize travel.

  injectRange(900, true);
  command = updateAt(navigation, centerStart + 1);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 900);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testStaleForwardRangeStopsCruise() {
  ObstacleAvoidance navigation;
  const uint32_t frontStart = armAndStartCenterMeasurement(navigation);

  injectRange(900, true);
  NavigationCommand command = updateAt(navigation, frontStart + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));

  // The sample is conservatively timestamped when the trigger fires.
  // Age == maxAge is fresh; age == maxAge + 1 must stop forward travel.
  command = updateAt(navigation, frontStart + FRONT_RANGE_MAX_AGE_MS);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));

  command = updateAt(navigation, frontStart + FRONT_RANGE_MAX_AGE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
}

int main() {
  testInvalidFrontRangeHoldsUntilSensorRecovers();
  testTurnRequiresFreshFrontRangeAfterward();
  testBothBlockedSidesHoldAndRescan();
  testSideEchoIsDiscardedWhenAutonomyIsDisabled();
  testSideEchoArrivingAfterRearmIsDiscarded();
  testStaleForwardRangeStopsCruise();
  return 0;
}
