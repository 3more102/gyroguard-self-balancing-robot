#include "../firmware/GyroGuard/arm_guard.h"
#include <cassert>
#include <limits>

int main() {
  const float limit = 8.0f;
  assert(canArmBalance(false, true, 0.0f, limit));
  assert(canArmBalance(false, true, 8.0f, limit));
  assert(canArmBalance(false, true, -8.0f, limit));
  assert(!canArmBalance(false, true, 8.01f, limit));
  assert(!canArmBalance(false, true, -8.01f, limit));
  assert(!canArmBalance(true, true, 0.0f, limit));
  assert(!canArmBalance(false, false, 0.0f, limit));
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  assert(!canArmBalance(false, true, nan, limit));
  assert(!canArmBalance(false, true, infinity, limit));
  assert(!canArmBalance(false, true, 0.0f, nan));
  assert(!canArmBalance(false, true, 0.0f, -1.0f));
  return 0;
}
