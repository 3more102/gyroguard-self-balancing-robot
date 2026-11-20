#include "../firmware/GyroGuard/battery_policy.h"
#include <cassert>
#include <cstdint>

int main() {
  assert(batteryMillivoltsFromAdc(0, 5000, 2) == 0U);
  assert(batteryMillivoltsFromAdc(860, 5000, 2) == 8407U);
  assert(batteryMillivoltsFromAdc(1023, 5000, 2) == 10000U);
  assert(batteryMillivoltsFromAdc(2048, 5000, 2) == 10000U);

  // Missing, shorted or over-range divider readings are not valid battery data.
  assert(!batteryVoltagePlausible(0, 5000, 9000));
  assert(!batteryVoltagePlausible(4999, 5000, 9000));
  assert(batteryVoltagePlausible(5000, 5000, 9000));
  assert(batteryVoltagePlausible(8400, 5000, 9000));
  assert(batteryVoltagePlausible(9000, 5000, 9000));
  assert(!batteryVoltagePlausible(9001, 5000, 9000));

  // Assert at/below the low threshold; do not assert just above it.
  assert(batteryTravelInhibited(false, 6800, 6800, 7000));
  assert(!batteryTravelInhibited(false, 6801, 6800, 7000));

  // Once inhibited, require the higher recovery threshold.
  assert(batteryTravelInhibited(true, 6800, 6800, 7000));
  assert(batteryTravelInhibited(true, 6999, 6800, 7000));
  assert(!batteryTravelInhibited(true, 7000, 6800, 7000));

  // A recovered battery remains clear above the inhibit threshold.
  assert(!batteryTravelInhibited(false, 7400, 6800, 7000));
  return 0;
}
