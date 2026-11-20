#include "../firmware/GyroGuard/sonar_math.h"
#include <cassert>
#include <cstdint>

int main() {
  const SonarReading noPulse = sonarReadingFromPulse(0U, 30U, 2500U);
  assert(!noPulse.valid);

  // Any captured near-field echo is treated as an obstruction.
  const SonarReading nearField = sonarReadingFromPulse(100U, 30U, 2500U);
  assert(nearField.valid && nearField.distanceMm == 0U);

  const SonarReading minBoundary = sonarReadingFromPulse(175U, 30U, 2500U);
  assert(minBoundary.valid && minBoundary.distanceMm == 30U);

  const SonarReading normal = sonarReadingFromPulse(1167U, 30U, 2500U);
  assert(normal.valid && normal.distanceMm == 200U);

  const SonarReading maxBoundary = sonarReadingFromPulse(14578U, 30U, 2500U);
  assert(maxBoundary.valid && maxBoundary.distanceMm == 2500U);

  const SonarReading beyondRange = sonarReadingFromPulse(14579U, 30U, 2500U);
  assert(!beyondRange.valid);

  const SonarReading captureTimeout = sonarReadingFromPulse(30001U, 30U, 2500U);
  assert(!captureTimeout.valid);

  const SonarReading invalidLimits = sonarReadingFromPulse(1000U, 300U, 250U);
  assert(!invalidLimits.valid);
  return 0;
}
