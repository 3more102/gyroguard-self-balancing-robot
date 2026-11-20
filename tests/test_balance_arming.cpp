#include "../firmware/GyroGuard/arm_guard.h"
#include <cassert>
#include <limits>
#include <stdint.h>

static bool canArmWithFreshSample(bool faultLatched, bool pitchValid,
                                  float pitchDeg, float pitchRateDps,
                                  float maxAbsPitchDeg,
                                  float maxAbsPitchRateDps) {
  return canArmBalance(faultLatched, pitchValid, pitchDeg, pitchRateDps,
                       maxAbsPitchDeg, maxAbsPitchRateDps,
                       100000U, 105000U, 15000U);
}

int main() {
  const float tiltLimit = 8.0f;
  const float rateLimit = 10.0f;

  assert(canArmWithFreshSample(false, true, 0.0f, 0.0f, tiltLimit, rateLimit));
  assert(canArmWithFreshSample(false, true, 8.0f, 10.0f, tiltLimit, rateLimit));
  assert(canArmWithFreshSample(false, true, -8.0f, -10.0f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, 8.01f, 0.0f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, -8.01f, 0.0f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, 0.0f, 10.01f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, 0.0f, -10.01f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(true, true, 0.0f, 0.0f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, false, 0.0f, 0.0f, tiltLimit, rateLimit));

  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float infinity = std::numeric_limits<float>::infinity();
  assert(!canArmWithFreshSample(false, true, nan, 0.0f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, 0.0f, nan, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, infinity, 0.0f, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, 0.0f, infinity, tiltLimit, rateLimit));
  assert(!canArmWithFreshSample(false, true, 0.0f, 0.0f, nan, rateLimit));
  assert(!canArmWithFreshSample(false, true, 0.0f, 0.0f, tiltLimit, nan));
  assert(!canArmWithFreshSample(false, true, 0.0f, 0.0f, -1.0f, rateLimit));
  assert(!canArmWithFreshSample(false, true, 0.0f, 0.0f, tiltLimit, -1.0f));

  // The 15 ms freshness boundary is inclusive; one microsecond older is stale.
  assert(isPitchSampleFresh(100000U, 115000U, 15000U));
  assert(!isPitchSampleFresh(100000U, 115001U, 15000U));
  assert(canArmBalance(false, true, 0.0f, 0.0f, tiltLimit, rateLimit,
                       100000U, 115000U, 15000U));
  assert(!canArmBalance(false, true, 0.0f, 0.0f, tiltLimit, rateLimit,
                        100000U, 115001U, 15000U));

  // Unsigned age arithmetic must remain valid when micros() wraps at 2^32.
  assert(isPitchSampleFresh(0xFFFFFF00UL, 0x00000200UL, 1000U));
  assert(!isPitchSampleFresh(0xFFFFFF00UL, 0x00000400UL, 1000U));
  // A future sample timestamp is rejected as an implausibly old sample.
  assert(!isPitchSampleFresh(200U, 199U, 15000U));
  return 0;
}
