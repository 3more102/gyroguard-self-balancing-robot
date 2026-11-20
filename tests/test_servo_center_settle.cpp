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

static void testRearmWaitsForCenterServoSettle() {
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
  injectRange(900, true);
  command = step(navigation, SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, CRUISE_SPEED_MM_S));
}

static void testTurnWaitsForCenterServoSettleBeforeForwardRange() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  navigation.setAutonomous(true);

  step(navigation, SERVO_SETTLE_MS);
  injectRange(250, true);
  step(navigation, SERVO_SETTLE_MS + 1);

  step(navigation, SERVO_SETTLE_MS + 1 + SERVO_SETTLE_MS);
  injectRange(800, true);
  step(navigation, SERVO_SETTLE_MS + 2 + SERVO_SETTLE_MS);

  step(navigation, SERVO_SETTLE_MS + 2 + 2 * SERVO_SETTLE_MS);
  injectRange(300, true);
  NavigationCommand command = step(
      navigation, SERVO_SETTLE_MS + 3 + 2 * SERVO_SETTLE_MS);
  assert(command.turnBiasPwm < 0.0f);

  const uint32_t turnFinished =
      SERVO_SETTLE_MS + 3 + 2 * SERVO_SETTLE_MS + TURN_DURATION_MS;
  command = step(navigation, turnFinished);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(!sonarBusy);

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

static void testCenterRangeBelowClearanceTriggersRescan() {
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

  const uint32_t holdStarted =
      SERVO_SETTLE_MS + 3 + 2 * SERVO_SETTLE_MS;
  command = step(navigation, holdStarted + SERVO_SETTLE_MS);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(sonarBusy);

  // 400 mm is above the stop threshold but below OBSTACLE_CLEAR_MM.
  injectRange(400, true);
  command = step(navigation, holdStarted + SERVO_SETTLE_MS + 1);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(fakeServoAngle == SERVO_LEFT_DEG);
}

static void testBalanceOnlyKeepsFrontSonarTelemetryWithoutTravel() {
  ObstacleAvoidance navigation;
  fakeMillis = 0;
  navigation.begin();
  assert(!navigation.autonomous());

  NavigationCommand command = step(navigation, SERVO_SETTLE_MS - 1);
  assert(!sonarBusy);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(near(command.turnBiasPwm, 0.0f));

  command = step(navigation, SERVO_SETTLE_MS);
  assert(sonarBusy);
  injectRange(800, true);
  command = step(navigation, SERVO_SETTLE_MS + 1);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 800);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(near(command.turnBiasPwm, 0.0f));

  const uint32_t nextSample = SERVO_SETTLE_MS + SONAR_RETRY_MS;
  command = step(navigation, nextSample);
  assert(sonarBusy);
  injectRange(700, true);
  command = step(navigation, nextSample + 1);
  assert(navigation.frontDistanceValid());
  assert(navigation.frontDistanceMm() == 700);
  assert(near(command.targetSpeedMmS, 0.0f));
  assert(near(command.turnBiasPwm, 0.0f));
}

int main() {
  testBalanceOnlyKeepsFrontSonarTelemetryWithoutTravel();
  testRearmWaitsForCenterServoSettle();
  testTurnWaitsForCenterServoSettleBeforeForwardRange();
  testCenterRangeBelowClearanceTriggersRescan();
  return 0;
}
