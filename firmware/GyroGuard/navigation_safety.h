#pragma once

#include <stdint.h>

// A range is safe for cruise only while it is valid, recent and outside the
// configured stop distance. Unsigned age arithmetic is wrap-safe for millis().
inline bool gyroguardFrontRangeAllowsCruise(bool rangeValid,
                                            uint16_t distanceMm,
                                            uint32_t nowMs,
                                            uint32_t lastMeasurementMs,
                                            uint32_t maxAgeMs,
                                            uint16_t stopDistanceMm) {
  const uint32_t ageMs = nowMs - lastMeasurementMs;
  return rangeValid && ageMs <= maxAgeMs && distanceMm >= stopDistanceMm;
}

enum class GyroGuardScanAction : uint8_t {
  Hold,
  TurnLeft,
  TurnRight,
  ReverseAndRescan
};

// Unknown is not free space. Turn toward a single known-clear side, hold if
// the only known side is blocked, and reverse/rescan only when both sides were
// measured successfully but neither has the configured clearance.
inline GyroGuardScanAction gyroguardChooseScanAction(bool leftValid,
                                                      uint16_t leftDistanceMm,
                                                      bool rightValid,
                                                      uint16_t rightDistanceMm,
                                                      uint16_t clearDistanceMm) {
  if (!leftValid && !rightValid) return GyroGuardScanAction::Hold;

  if (!leftValid) {
    return rightDistanceMm >= clearDistanceMm
        ? GyroGuardScanAction::TurnRight
        : GyroGuardScanAction::Hold;
  }

  if (!rightValid) {
    return leftDistanceMm >= clearDistanceMm
        ? GyroGuardScanAction::TurnLeft
        : GyroGuardScanAction::Hold;
  }

  if (leftDistanceMm < clearDistanceMm &&
      rightDistanceMm < clearDistanceMm) {
    return GyroGuardScanAction::ReverseAndRescan;
  }

  return leftDistanceMm > rightDistanceMm
      ? GyroGuardScanAction::TurnLeft
      : GyroGuardScanAction::TurnRight;
}
