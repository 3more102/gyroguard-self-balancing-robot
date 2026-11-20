#include "../firmware/GyroGuard/encoder_math.h"
#include <cassert>
#include <cmath>

static bool near(float a, float b, float eps = 1.0e-4f) {
  return std::fabs(a - b) <= eps;
}

int main() {
  // 200 mm circumference / 100 counts per revolution = 2 mm per count.
  assert(near(encoderCountsToSpeedMmS(10, 10, 200.0f, 100.0f, 1.0f),
              20.0f));

  // Average the two wheels so a one-wheel delta is not double-counted.
  assert(near(encoderCountsToSpeedMmS(10, 0, 200.0f, 100.0f, 1.0f),
              10.0f));

  // Reverse travel remains negative.
  assert(near(encoderCountsToSpeedMmS(-10, -10, 200.0f, 100.0f, 1.0f),
              -20.0f));

  // Opposite wheel motion has zero average linear velocity.
  assert(near(encoderCountsToSpeedMmS(10, -10, 200.0f, 100.0f, 1.0f),
              0.0f));

  // Invalid physical parameters or elapsed time fail closed to zero.
  assert(near(encoderCountsToSpeedMmS(10, 10, 200.0f, 100.0f, 0.0f),
              0.0f));
  assert(near(encoderCountsToSpeedMmS(10, 10, 200.0f, 100.0f, -1.0f),
              0.0f));
  assert(near(encoderCountsToSpeedMmS(10, 10, 0.0f, 100.0f, 1.0f),
              0.0f));
  assert(near(encoderCountsToSpeedMmS(10, 10, 200.0f, 0.0f, 1.0f),
              0.0f));

  return 0;
}
