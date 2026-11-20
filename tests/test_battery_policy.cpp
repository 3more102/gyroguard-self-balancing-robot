#include "../firmware/GyroGuard/battery_policy.h"
#include <cassert>
#include <cstdint>

int main() {
  assert(batteryMillivoltsFromAdc(0, 5000, 2) == 0U);
  assert(batteryMillivoltsFromAdc(860, 5000, 2) == 8407U);
  assert(batteryMillivoltsFromAdc(1023, 5000, 2) == 10000U);
  assert(batteryMillivoltsFromAdc(2048, 5000, 2) == 10000U);
  assert(batteryMillivoltsFromAdc(1023, 65535, 255) == 65535U);
  assert(batteryMillivoltsFromAdc(1023, 5000, 0) == 0U);

  // Missing, shorted or over-range divider readings are not valid battery data.
  assert(!batteryVoltagePlausible(0, 5000, 9000));
  assert(!batteryVoltagePlausible(4999, 5000, 9000));
  assert(batteryVoltagePlausible(5000, 5000, 9000));
  assert(batteryVoltagePlausible(8400, 5000, 9000));
  assert(batteryVoltagePlausible(9000, 5000, 9000));
  assert(!batteryVoltagePlausible(9001, 5000, 9000));

  // Assert at/below the low threshold; do not assert just above it.
  assert(batteryTravelInhibited(false, 6800, 8000, true, 6800, 7000));
  assert(!batteryTravelInhibited(false, 6801, 8000, true, 6800, 7000));

  // Once inhibited, both raw and filtered readings must reach recovery.
  assert(batteryTravelInhibited(true, 8000, 6999, true, 6800, 7000));
  assert(batteryTravelInhibited(true, 6999, 8000, true, 6800, 7000));
  assert(!batteryTravelInhibited(true, 7000, 7000, true, 6800, 7000));

  // Invalid samples fail closed and cannot clear a previously latched inhibit.
  assert(batteryTravelInhibited(true, 10000, 8000, false, 6800, 7000));
  assert(batteryTravelInhibited(true, 6900, 8000, true, 6800, 7000));
  assert(batteryTravelInhibited(false, 10000, 8000, false, 6800, 7000));
  assert(batteryTravelInhibited(true, 6900, 6999, true, 6800, 7000));

  // Once recovered, the lower threshold provides hysteresis.
  assert(!batteryTravelInhibited(false, 6900, 6900, true, 6800, 7000));
  assert(batteryTravelInhibited(false, 6800, 8000, true, 6800, 7000));
  return 0;
}
