#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "sonar.h"

struct NavigationCommand {
  float targetSpeedMmS;
  float turnBiasPwm;
};

class ObstacleAvoidance {
public:
  void begin();
  void setAutonomous(bool enabled);
  bool autonomous() const;
  bool frontDistanceValid() const;
  uint16_t frontDistanceMm() const;
  NavigationCommand update(uint32_t nowMs, uint32_t nowUs);

private:
  enum State : uint8_t {
    DRIVE,
    SCAN_LEFT_SETTLE,
    SCAN_LEFT_MEASURE,
    SCAN_RIGHT_SETTLE,
    SCAN_RIGHT_MEASURE,
    TURNING,
    REVERSING,
    SAFE_STOP
  };

  void startScan(uint32_t nowMs);
  void startSonar(uint32_t nowUs, uint32_t nowMs, State measureState);
  void consumeSonar(uint32_t nowMs);

  Servo servo_;
  Ultrasonic sonar_;
  State state_;
  bool autonomous_;
  uint32_t stateSinceMs_;
  uint32_t lastSonarStartMs_;
  uint16_t leftDistanceMm_;
  uint16_t rightDistanceMm_;
  uint16_t frontDistanceMm_;
  bool frontDistanceValid_;
  bool haveLeft_;
  bool haveRight_;
  float turnDirection_;
};
