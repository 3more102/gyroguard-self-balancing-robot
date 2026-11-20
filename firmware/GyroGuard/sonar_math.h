#pragma once

#include <stdint.h>

// Convert an HC-SR04 echo pulse to a conservative range result.
// A nonzero echo shorter than the sensor's reliable range is a 0 mm
// obstruction, not an invalid/clear path. The capture timeout is 30 ms.
struct SonarReading {
  uint16_t distanceMm;
  bool valid;
};

inline SonarReading sonarReadingFromPulse(uint32_t pulseUs,
                                          uint16_t minRangeMm,
                                          uint16_t maxRangeMm) {
  if (pulseUs == 0U || pulseUs > 30000UL || minRangeMm > maxRangeMm) {
    return {0U, false};
  }

  // pulseUs is bounded to 30,000 us, so this multiplication fits uint32_t
  // on the ATmega328P and avoids costly 64-bit arithmetic.
  const uint32_t distanceMm = (pulseUs * 343UL) / 2000UL;
  if (distanceMm > maxRangeMm) return {0U, false};
  if (distanceMm < minRangeMm) return {0U, true};
  return {static_cast<uint16_t>(distanceMm), true};
}
