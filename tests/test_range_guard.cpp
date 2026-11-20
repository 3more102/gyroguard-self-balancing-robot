#include "../firmware/GyroGuard/range_guard.h"
#include <cassert>
#include <stdint.h>

int main() {
  assert(isRangeSampleFresh(true, 100U, 100U, 150U));
  assert(isRangeSampleFresh(true, 100U, 250U, 150U));
  assert(!isRangeSampleFresh(true, 100U, 251U, 150U));
  assert(!isRangeSampleFresh(false, 100U, 100U, 150U));

  // millis() wraps at UINT32_MAX; unsigned subtraction must preserve age.
  const uint32_t sampledAt = UINT32_MAX - 20U;
  assert(isRangeSampleFresh(true, sampledAt, 50U, 100U));
  assert(!isRangeSampleFresh(true, sampledAt, 50U, 70U));
  return 0;
}
