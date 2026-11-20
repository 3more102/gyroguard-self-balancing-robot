#pragma once

#include <stdint.h>

// Convert an HC-SR04 echo pulse to a conservative range result.
// A nonzero echo shorter than the sensor's configured reliable range is
// reported as a valid 0 mm obstruction, not discarded as "no obstacle".
struct SonarReading {
  uint16_t distanceMm;
  bool valid;
};

inline SonarReading sonarReadingFromPulse(uint32_t pulseUs,
                                          uint16_t minRangeMm,
                                          uint16_t maxRangeMm) {
  if (pulseUs == 0U || minRangeMm > maxRangeMm) {
    return {0U, false};
  }

  const uint64_t scaled = static_cast<uint64_t>(pulseUs) * 343ULL;
  const uint32_t distanceMm = static_cast<uint32_t>(scaled / 2000ULL);

  if (distanceMm > maxRangeMm) {
    return {0U, false};
  }
  if (distanceMm < minRangeMm) {
    return {0U, true};
  }

  return {static_cast<uint16_t>(distanceMm), true};
}
