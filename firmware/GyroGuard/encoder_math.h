#pragma once

// Pure encoder-to-linear-velocity conversion, kept independent of Arduino APIs
// so the same calculation can be covered by host-side unit tests.
inline float encoderCountsToSpeedMmS(long leftDelta, long rightDelta,
                                     float wheelCircumferenceMm,
                                     float countsPerWheelRev,
                                     float elapsedSeconds) {
  if (!(wheelCircumferenceMm > 0.0f) ||
      !(countsPerWheelRev > 0.0f) ||
      !(elapsedSeconds > 0.0f)) {
    return 0.0f;
  }

  const float averageCounts =
      0.5f * (static_cast<float>(leftDelta) +
              static_cast<float>(rightDelta));
  return averageCounts * wheelCircumferenceMm /
         (countsPerWheelRev * elapsedSeconds);
}
