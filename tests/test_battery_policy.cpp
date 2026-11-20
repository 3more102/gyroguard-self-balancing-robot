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

  assert(!batteryVoltagePlausible(0, 5000, 9000));
  assert(!batteryVoltagePlausible(4999, 5000, 9000));
  assert(batteryVoltagePlausible(5000, 5000, 9000));
  assert(batteryVoltagePlausible(8400, 5000, 9000));
  assert(batteryVoltagePlausible(9000, 5000, 9000));
  assert(!batteryVoltagePlausible(9001, 5000, 9000));

  assert(batteryUpdateTravelInhibit(false, 6800, 6800, true, 6800, 7000));
  assert(!batteryUpdateTravelInhibit(false, 6801, 6801, true, 6800, 7000));

  assert(batteryUpdateTravelInhibit(true, 6999, 7500, true, 6800, 7000));
  assert(batteryUpdateTravelInhibit(true, 7000, 6999, true, 6800, 7000));
  assert(!batteryUpdateTravelInhibit(true, 7000, 7000, true, 6800, 7000));

  // Invalid raw samples block travel separately and cannot clear a prior latch.
  assert(!batteryUpdateTravelInhibit(false, 0, 8400, false, 6800, 7000));
  assert(batteryUpdateTravelInhibit(true, 10000, 6800, false, 6800, 7000));
  assert(!batteryUpdateTravelInhibit(false, 7400, 7400, true, 6800, 7000));
  return 0;
}
