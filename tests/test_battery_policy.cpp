#include "../firmware/GyroGuard/battery_policy.h"
#include <cassert>
#include <cstdint>

int main() {
  assert(batteryMillivoltsFromAdc(0, 5000, 2) == 0U);
  assert(batteryMillivoltsFromAdc(860, 5000, 2) == 8407U);
  assert(batteryMillivoltsFromAdc(1023, 5000, 2) == 10000U);
  assert(batteryMillivoltsFromAdc(2048, 5000, 2) == 10000U);

  // Missing, shorted or over-range divider readings are not valid data.
  assert(!batteryVoltagePlausible(0, 5000, 9000));
  assert(!batteryVoltagePlausible(4999, 5000, 9000));
  assert(batteryVoltagePlausible(5000, 5000, 9000));
  assert(batteryVoltagePlausible(8400, 5000, 9000));
  assert(batteryVoltagePlausible(9000, 5000, 9000));
  assert(!batteryVoltagePlausible(9001, 5000, 9000));

  // Initial/invalid states fail closed and require a valid recovery voltage.
  assert(batteryTravelInhibited(true, 6900, true, 6800, 7000));
  assert(!batteryTravelInhibited(true, 7000, true, 6800, 7000));
  assert(batteryTravelInhibited(false, 0, false, 6800, 7000));
  assert(batteryTravelInhibited(true, 9100, false, 6800, 7000));

  // Assert at/below the low threshold; do not assert just above it.
  assert(batteryTravelInhibited(false, 6800, true, 6800, 7000));
  assert(!batteryTravelInhibited(false, 6801, true, 6800, 7000));

  // Once inhibited, require the higher recovery threshold.
  assert(batteryTravelInhibited(true, 6800, true, 6800, 7000));
  assert(batteryTravelInhibited(true, 6999, true, 6800, 7000));
  assert(!batteryTravelInhibited(true, 7000, true, 6800, 7000));

  // An invalid sample during an inhibit must not clear the latch.
  assert(batteryTravelInhibited(true, 9500, false, 6800, 7000));
  assert(batteryTravelInhibited(true, 6900, true, 6800, 7000));

  // Once recovered, the lower threshold provides hysteresis.
  assert(!batteryTravelInhibited(false, 6900, true, 6800, 7000));
  assert(batteryTravelInhibited(false, 6800, true, 6800, 7000));
  return 0;
}
