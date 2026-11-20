#pragma once

#include <stdint.h>

// Divider ratio is (Rtop + Rbottom) / Rbottom. Equal 10 kΩ resistors give 2.
// Saturate on extreme caller-supplied values rather than allowing uint32_t wrap.
inline uint16_t batteryMillivoltsFromAdc(uint16_t adcReading,
                                         uint16_t referenceMv,
                                         uint8_t dividerRatio) {
  if (adcReading > 1023U) adcReading = 1023U;
  const uint32_t base = static_cast<uint32_t>(adcReading) * referenceMv;
  if (dividerRatio != 0U &&
      base > (0xFFFFFFFFUL - 511UL) / dividerRatio) {
    return 0xFFFFU;
  }
  const uint32_t numerator = base * dividerRatio;
  const uint32_t result = (numerator + 511UL) / 1023UL;
  return result > 0xFFFFUL ? 0xFFFFU : static_cast<uint16_t>(result);
}

inline bool batteryVoltagePlausible(uint16_t batteryMv,
                                    uint16_t minimumMv,
                                    uint16_t maximumMv) {
  return batteryMv >= minimumMv && batteryMv <= maximumMv;
}

// Apply hysteresis so load transients do not repeatedly arm/disarm autonomy.
inline bool batteryTravelInhibited(bool currentlyInhibited,
                                   uint16_t batteryMv,
                                   uint16_t inhibitBelowOrAtMv,
                                   uint16_t recoverAtOrAboveMv) {
  if (currentlyInhibited) return batteryMv < recoverAtOrAboveMv;
  return batteryMv <= inhibitBelowOrAtMv;
}
