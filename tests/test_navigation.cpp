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

bool Ultrasonic::busy() const {
  return sonarBusy;
}

static void testFreshFrontRangeRequiredBeforeCruise() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  NavigationCommand command = updateAt(navigation, SONAR_RETRY_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(700);
  command = updateAt(navigation, SONAR_RETRY_MS + 1);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 700);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testInvalidFrontRangeHoldsUntilRecovery() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(0, false);
  NavigationCommand command = updateAt(navigation, SONAR_RETRY_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  command = updateAt(navigation, 2 * SONAR_RETRY_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(700);
  command = updateAt(navigation, 2 * SONAR_RETRY_MS + 2);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 700);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testObstacleScanAndTurnRequiresNewFrontReading() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(250);
  NavigationCommand command = updateAt(navigation, SONAR_RETRY_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(fakeServoAngle == SERVO_LEFT_DEG);

  updateAt(navigation, SONAR_RETRY_MS + 1 + SERVO_SETTLE_MS);
  assert(sonarBusy);
  injectRange(600);
  updateAt(navigation, SONAR_RETRY_MS + 2 + SERVO_SETTLE_MS);
  assert(fakeServoAngle == SERVO_RIGHT_DEG);

  updateAt(navigation, SONAR_RETRY_MS + 2 + 2 * SERVO_SETTLE_MS);
  assert(sonarBusy);
  injectRange(350);
  command = updateAt(navigation, SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS);
  assert(fakeServoAngle == SERVO_CENTER_DEG);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(command.turnBiasPwm < 0.0f); // More measured clearance on the left.

  const uint32_t turnFinishedMs =
      SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS + TURN_DURATION_MS;
  command = updateAt(navigation, turnFinishedMs);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());

  command = updateAt(navigation, turnFinishedMs + SONAR_RETRY_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);
  injectRange(700);
  command = updateAt(navigation, turnFinishedMs + SONAR_RETRY_MS + 1);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 700);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testBothBlockedSidesHoldAndReacquireCenterRange() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(200);
  updateAt(navigation, SONAR_RETRY_MS + 1);

  updateAt(navigation, SONAR_RETRY_MS + 1 + SERVO_SETTLE_MS);
  injectRange(200);
  updateAt(navigation, SONAR_RETRY_MS + 2 + SERVO_SETTLE_MS);

  updateAt(navigation, SONAR_RETRY_MS + 2 + 2 * SERVO_SETTLE_MS);
  injectRange(250);
  NavigationCommand command =
      updateAt(navigation, SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(fakeServoAngle == SERVO_CENTER_DEG);

  const uint32_t holdRetryMs =
      SONAR_RETRY_MS + 3 + 2 * SERVO_SETTLE_MS + SONAR_RETRY_MS;
  command = updateAt(navigation, holdRetryMs);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  injectRange(700);
  command = updateAt(navigation, holdRetryMs + 1);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 700);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testStopDuringSideScanDoesNotReuseItsRangeAfterRearm() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  updateAt(navigation, SONAR_RETRY_MS);
  injectRange(200);
  updateAt(navigation, SONAR_RETRY_MS + 1);

  updateAt(navigation, SONAR_RETRY_MS + 1 + SERVO_SETTLE_MS);
  assert(sonarBusy); // Left-side measurement is now in flight.

  navigation.setAutonomous(false);
  assert(!navigation.autonomous());
  assert(fakeServoAngle == SERVO_CENTER_DEG);
  navigation.setAutonomous(true);

  // This is the old left-side result. It must be discarded after re-arming.
  injectRange(900);
  NavigationCommand command =
      updateAt(navigation, SONAR_RETRY_MS + 2 + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!navigation.frontDistanceValid());
  assert(sonarBusy); // A new center measurement has started.

  injectRange(700);
  command = updateAt(navigation, SONAR_RETRY_MS + 3 + SERVO_SETTLE_MS);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 700);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testManualStopDisablesTravel() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);
  navigation.setAutonomous(false);

  const NavigationCommand command = updateAt(navigation, 1000);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(near(command.turnBiasPwm, 0.0f));
  assert(!navigation.autonomous());
  assert(fakeServoAngle == SERVO_CENTER_DEG);
}

int main() {
  testFreshFrontRangeRequiredBeforeCruise();
  testInvalidFrontRangeHoldsUntilRecovery();
  testObstacleScanAndTurnRequiresNewFrontReading();
  testBothBlockedSidesHoldAndReacquireCenterRange();
  testStopDuringSideScanDoesNotReuseItsRangeAfterRearm();
  testManualStopDisablesTravel();
  return 0;
}
