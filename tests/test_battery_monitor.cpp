#include "../firmware/GyroGuard/battery_monitor.h"
#include "../firmware/GyroGuard/config.h"

#include <cassert>
#include <cstdint>

uint16_t fakeAnalogReadValue = 860;  // Approximately 8.4 V through a 10k/10k divider.

int main() {
  BatteryMonitor monitor;
  monitor.begin();

  assert(monitor.update(0));
  assert(monitor.ready());
  assert(monitor.voltageValid());
  assert(!monitor.lowBattery());
  assert(monitor.millivolts() == 8407U);
  assert(!monitor.travelInhibited());
  assert(!monitor.update(BATTERY_SAMPLE_PERIOD_MS - 1));

  // A low raw sample asserts the inhibit immediately despite filter lag.
  fakeAnalogReadValue = 680;  // Approximately 6.65 V.
  assert(monitor.update(BATTERY_SAMPLE_PERIOD_MS));
  assert(monitor.voltageValid());
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());

  // An invalid high sample must not clear the existing low-voltage latch
  // or contaminate the last valid filtered reading.
  const uint16_t lastFiltered = monitor.millivolts();
  fakeAnalogReadValue = 1023;  // 10.0 V is outside the configured 2S range.
  assert(monitor.update(BATTERY_SAMPLE_PERIOD_MS * 2U));
  assert(!monitor.voltageValid());
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());
  assert(monitor.millivolts() == lastFiltered);

  // A valid reading between 6.8 V and 7.0 V does not clear the latch.
  fakeAnalogReadValue = 700;  // Approximately 6.84 V.
  assert(monitor.update(BATTERY_SAMPLE_PERIOD_MS * 3U));
  assert(monitor.voltageValid());
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());

  // Recovery requires a valid raw sample and filtered value >= 7.0 V.
  fakeAnalogReadValue = 717;  // Approximately 7.01 V.
  assert(monitor.update(BATTERY_SAMPLE_PERIOD_MS * 4U));
  assert(monitor.voltageValid());
  assert(!monitor.lowBattery());
  assert(!monitor.travelInhibited());
  return 0;
}
