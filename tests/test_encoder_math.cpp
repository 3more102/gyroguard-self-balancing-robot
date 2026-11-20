#include "../firmware/GyroGuard/encoder_math.h"
#include <cassert>
#include <cmath>

int main() {
  const float forward = encoderCountsToSpeedMmS(100, 100, 204.2035f, 660.0f, 1.0f);
  assert(std::fabs(forward - (100.0f * 204.2035f / 660.0f)) < 0.001f);

  const float reverse = encoderCountsToSpeedMmS(-100, -100, 204.2035f, 660.0f, 1.0f);
  assert(std::fabs(reverse + forward) < 0.001f);

  const float unequal = encoderCountsToSpeedMmS(100, 50, 200.0f, 500.0f, 0.5f);
  assert(std::fabs(unequal - 60.0f) < 0.001f);

  assert(encoderCountsToSpeedMmS(10, 10, 0.0f, 500.0f, 1.0f) == 0.0f);
  assert(encoderCountsToSpeedMmS(10, 10, 200.0f, 0.0f, 1.0f) == 0.0f);
  assert(encoderCountsToSpeedMmS(10, 10, 200.0f, 500.0f, 0.0f) == 0.0f);
  return 0;
}
