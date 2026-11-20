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
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  NavigationCommand command = updateAt(navigation, SONAR_RETRY_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(0, false);
  command = updateAt(navigation, SONAR_RETRY_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  command = updateAt(navigation, 2 * SONAR_RETRY_MS + 1);
  assert(sonarBusy);
  injectRange(900, true);
  command = updateAt(navigation, 2 * SONAR_RETRY_MS + 2);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 900);
}

static void testTurnRequiresFreshFrontRangeAfterward() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(250, true);
  NavigationCommand command = updateAt(navigation, SONAR_RETRY_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(fakeServoAngle == SERVO_LEFT_DEG);

  updateAt(navigation, SONAR_RETRY_MS + 1 + SERVO_SETTLE_MS);
  injectRange(800, true);
  updateAt(navigation, SONAR_RETRY_MS + 2 + SERVO_SETTLE_MS);
  assert(fakeServoAngle == SERVO_RIGHT_DEG);

  updateAt(navigation, SONAR_RETRY_MS + 2 + 2 * SERVO_SETTLE_MS);
  injectRange(300, true);
  command = updateAt(navigation, SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS);
  assert(fakeServoAngle == SERVO_CENTER_DEG);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(command.turnBiasPwm < 0.0f); // The left side has verified clearance.

  const uint32_t turnFinished =
      SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS + TURN_DURATION_MS;
  command = updateAt(navigation, turnFinished);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  command = updateAt(navigation, turnFinished + SONAR_RETRY_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);
  injectRange(900, true);
  command = updateAt(navigation, turnFinished + SONAR_RETRY_MS + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testBothBlockedSidesHoldAndRescan() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(250, true);
  updateAt(navigation, SONAR_RETRY_MS + 1);

  updateAt(navigation, SONAR_RETRY_MS + 1 + SERVO_SETTLE_MS);
  injectRange(200, true);
  updateAt(navigation, SONAR_RETRY_MS + 2 + SERVO_SETTLE_MS);

  updateAt(navigation, SONAR_RETRY_MS + 2 + 2 * SERVO_SETTLE_MS);
  injectRange(350, true);
  NavigationCommand command = updateAt(
      navigation, SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(near(command.turnBiasPwm, 0.0f));
  assert(fakeServoAngle == SERVO_CENTER_DEG);

  command = updateAt(
      navigation, SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS + SONAR_RETRY_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);
  injectRange(250, true);
  command = updateAt(
      navigation, SONAR_RETRY_MS + 4 + 2 * SERVO_SETTLE_MS + SONAR_RETRY_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(fakeServoAngle == SERVO_LEFT_DEG);
}

static void testSideEchoIsDiscardedWhenAutonomyIsDisabled() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(250, true);
  updateAt(navigation, SONAR_RETRY_MS + 1);

  updateAt(navigation, SONAR_RETRY_MS + 1 + SERVO_SETTLE_MS);
  assert(sonarBusy);
  navigation.setAutonomous(false);
  injectRange(800, true);
  NavigationCommand command = updateAt(
      navigation, SONAR_RETRY_MS + 2 + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(fakeServoAngle == SERVO_CENTER_DEG);

  navigation.setAutonomous(true);
  command = updateAt(navigation, SONAR_RETRY_MS + 3 + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);
  injectRange(900, true);
  command = updateAt(navigation, SONAR_RETRY_MS + 4 + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testStaleForwardRangeStopsCruise() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(900, true);
  NavigationCommand command = updateAt(navigation, SONAR_RETRY_MS + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));

  command = updateAt(
      navigation, SONAR_RETRY_MS + FRONT_RANGE_MAX_AGE_MS);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));

  command = updateAt(
      navigation, SONAR_RETRY_MS + FRONT_RANGE_MAX_AGE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
}

int main() {
  testInvalidFrontRangeHoldsUntilSensorRecovers();
  testTurnRequiresFreshFrontRangeAfterward();
  testBothBlockedSidesHoldAndRescan();
  testSideEchoIsDiscardedWhenAutonomyIsDisabled();
  testStaleForwardRangeStopsCruise();
  return 0;
}
