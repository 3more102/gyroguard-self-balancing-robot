#include "../firmware/GyroGuard/motion_guard.h"

#include <cassert>
#include <stdint.h>

int main() {
  constexpr uint32_t travelTimeoutUs = 750000UL;
  constexpr uint32_t turnTimeoutUs = 400000UL;

  assert(!encoderFeedbackTimedOut(
      900000UL, 1000UL, 1000UL, MOTION_DEMAND_NONE,
      travelTimeoutUs, turnTimeoutUs));

  assert(!encoderFeedbackTimedOut(
      500000UL, 100000UL, 100000UL, MOTION_DEMAND_TRANSLATION,
      travelTimeoutUs, turnTimeoutUs));
  assert(encoderFeedbackTimedOut(
      851000UL, 100000UL, 200000UL, MOTION_DEMAND_TRANSLATION,
      travelTimeoutUs, turnTimeoutUs));
  assert(encoderFeedbackTimedOut(
      951000UL, 200000UL, 100000UL, MOTION_DEMAND_TRANSLATION,
      travelTimeoutUs, turnTimeoutUs));

  // A turn can have one stationary wheel, but both encoders timing out is a fault.
  assert(!encoderFeedbackTimedOut(
      600000UL, 100000UL, 500000UL, MOTION_DEMAND_ROTATION,
      travelTimeoutUs, turnTimeoutUs));
  assert(encoderFeedbackTimedOut(
      600000UL, 100000UL, 150000UL, MOTION_DEMAND_ROTATION,
      travelTimeoutUs, turnTimeoutUs));

  // micros() wraps at 32 bits; elapsed-time checks must remain valid.
  const uint32_t beforeWrap = 0xFFFFFFFFUL - 100UL;
  assert(!elapsedAtLeast(200UL, beforeWrap, 351UL));
  assert(elapsedAtLeast(250UL, beforeWrap, 351UL));

  return 0;
}
