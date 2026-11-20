#include "../firmware/GyroGuard/arm_guard.h"
#include <cassert>
#include <limits>

int main() {
  const float tiltLimit = 8.0f;
  const float rateLimit = 10.0f;

  assert(canArmBalance(false, true, 0.0f, 0.0f, tiltLimit, rateLimit));
  assert(canArmBalance(false, true, 8.0f, 10.0f, tiltLimit, rateLimit));
  assert(canArmBalance(false, true, -8.0f, -10.0f, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, 8.01f, 0.0f, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, -8.01f, 0.0f, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, 0.0f, 10.01f, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, 0.0f, -10.01f, tiltLimit, rateLimit));
  assert(!canArmBalance(true, true, 0.0f, 0.0f, tiltLimit, rateLimit));
  assert(!canArmBalance(false, false, 0.0f, 0.0f, tiltLimit, rateLimit));

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  assert(!canArmBalance(false, true, nan, 0.0f, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, 0.0f, nan, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, infinity, 0.0f, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, 0.0f, infinity, tiltLimit, rateLimit));
  assert(!canArmBalance(false, true, 0.0f, 0.0f, nan, rateLimit));
  assert(!canArmBalance(false, true, 0.0f, 0.0f, tiltLimit, nan));
  assert(!canArmBalance(false, true, 0.0f, 0.0f, -1.0f, rateLimit));
  assert(!canArmBalance(false, true, 0.0f, 0.0f, tiltLimit, -1.0f));
  return 0;
}
