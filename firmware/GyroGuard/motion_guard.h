#pragma once

#include <stdint.h>

enum MotionDemandMode : uint8_t {
  MOTION_DEMAND_NONE,
  MOTION_DEMAND_TRANSLATION,
  MOTION_DEMAND_ROTATION
};

// Unsigned subtraction keeps elapsed-time checks correct across micros() rollover.
inline bool elapsedAtLeast(uint32_t now, uint32_t start, uint32_t interval) {
  return static_cast<uint32_t>(now - start) >= interval;
}

// Straight travel needs both encoders; during a differential turn, one wheel
// may legitimately remain stationary, so at least one encoder must report motion.
inline bool encoderFeedbackTimedOut(uint32_t nowUs,
                                    uint32_t lastLeftMotionUs,
                                    uint32_t lastRightMotionUs,
                                    MotionDemandMode demand,
                                    uint32_t travelTimeoutUs,
                                    uint32_t turnTimeoutUs) {
  if (demand == MOTION_DEMAND_TRANSLATION) {
    return elapsedAtLeast(nowUs, lastLeftMotionUs, travelTimeoutUs) ||
           elapsedAtLeast(nowUs, lastRightMotionUs, travelTimeoutUs);
  }
  if (demand == MOTION_DEMAND_ROTATION) {
    return elapsedAtLeast(nowUs, lastLeftMotionUs, turnTimeoutUs) &&
           elapsedAtLeast(nowUs, lastRightMotionUs, turnTimeoutUs);
  }
  return false;
}
