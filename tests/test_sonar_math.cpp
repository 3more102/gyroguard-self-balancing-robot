#include "../firmware/GyroGuard/sonar_math.h"
#include <cassert>

int main() {
  const SonarReading noPulse = sonarReadingFromPulse(0U, 30U, 2500U);
  assert(!noPulse.valid);

  // A captured echo inside the nominal blind zone must be treated as an
  // immediate obstruction, never as an invalid/clear path.
  const SonarReading nearField = sonarReadingFromPulse(100U, 30U, 2500U);
  assert(nearField.valid);
  assert(nearField.distanceMm == 0U);

  const SonarReading minBoundary = sonarReadingFromPulse(175U, 30U, 2500U);
  assert(minBoundary.valid);
  assert(minBoundary.distanceMm == 30U);

  const SonarReading normal = sonarReadingFromPulse(1167U, 30U, 2500U);
  assert(normal.valid);
  assert(normal.distanceMm == 200U);

  const SonarReading beyondRange = sonarReadingFromPulse(20000U, 30U, 2500U);
  assert(!beyondRange.valid);

  const SonarReading invalidLimits = sonarReadingFromPulse(1000U, 300U, 250U);
  assert(!invalidLimits.valid);

  return 0;
}
