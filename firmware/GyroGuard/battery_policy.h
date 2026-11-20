#pragma once

#include <stdint.h>

// Divider ratio is (Rtop + Rbottom) / Rbottom. For equal 10 kΩ
// resistors the ratio is 2. Use integer arithmetic for host-testability.
inline uint16_t batteryMillivoltsFromAdc(uint16_t adcReading,
                                         uint16_t referenceMv,
                                         uint8_t dividerRatio) {
  if (adcReading > 1023U) adcReading = 1023U;
  const uint32_t numerator =
      static_cast<uint32_t>(adcReading) * referenceMv * dividerRatio;
  return static_cast<uint16_t>((numerator + 511U) / 1023U);
}

inline bool batteryVoltagePlausible(uint16_t batteryMv,
                                    uint16_t minimumMv,
                                    uint16_t maximumMv) {
  return batteryMv >= minimumMv && batteryMv <= maximumMv;
}

// Invalid measurements always inhibit travel. Once inhibited, require the
// higher recovery threshold; after recovery, trip again at/below the low limit.
inline bool batteryTravelInhibited(bool currentlyInhibited,
                                   uint16_t batteryMv,
                                   bool voltageValid,
                                   uint16_t inhibitBelowOrAtMv,
                                   uint16_t recoverAtOrAboveMv) {
  if (!voltageValid) return true;
  if (currentlyInhibited) return batteryMv < recoverAtOrAboveMv;
  return batteryMv <= inhibitBelowOrAtMv;
}
