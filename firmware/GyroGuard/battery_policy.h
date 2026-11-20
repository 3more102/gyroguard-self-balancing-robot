#pragma once

#include <stdint.h>

// The divider ratio is (Rtop + Rbottom) / Rbottom. For equal 10 kΩ
// resistors this is 2. Use integer arithmetic to keep the policy host-testable.
inline uint16_t batteryMillivoltsFromAdc(uint16_t adcReading,
                                         uint16_t referenceMv,
                                         uint8_t dividerRatio) {
  if (adcReading > 1023U) adcReading = 1023U;
  const uint32_t numerator =
      static_cast<uint32_t>(adcReading) * referenceMv * dividerRatio;
  return static_cast<uint16_t>((numerator + 511U) / 1023U);
}

// Apply hysteresis so load transients do not repeatedly arm/disarm autonomy.
inline bool batteryTravelInhibited(bool currentlyInhibited,
                                   uint16_t batteryMv,
                                   uint16_t inhibitBelowOrAtMv,
                                   uint16_t recoverAtOrAboveMv) {
  if (currentlyInhibited) return batteryMv < recoverAtOrAboveMv;
  return batteryMv <= inhibitBelowOrAtMv;
}
