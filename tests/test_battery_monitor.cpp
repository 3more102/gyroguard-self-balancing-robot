#include "../firmware/GyroGuard/battery_monitor.h"
#include <cassert>
#include <cstdint>

uint32_t fakeMillis = 0;
uint16_t fakeAnalogValue = 860;

int main() {
  BatteryMonitor monitor;
  monitor.begin();
  assert(!monitor.ready());
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // A healthy 2S pack clears the startup inhibit.
  assert(monitor.update(0));
  assert(monitor.ready());
  assert(monitor.voltageValid());
  assert(!monitor.travelInhibited());
  assert(monitor.millivolts() == 8407U);

  // Sampling is rate-limited to 100 ms.
  fakeAnalogValue = 0;
  assert(!monitor.update(99));
  assert(!monitor.travelInhibited());

  // A raw over-range sample is invalid even while the IIR output looks valid.
  fakeAnalogValue = 1023;
  assert(monitor.update(100));
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // A valid healthy reading restores operation after the invalid sample.
  fakeAnalogValue = 860;
  assert(monitor.update(200));
  assert(monitor.voltageValid());
  assert(!monitor.travelInhibited());

  // A disconnected divider fails closed even if the filtered voltage remains
  // inside the nominal plausibility band.
  fakeAnalogValue = 0;
  assert(monitor.update(300));
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // Recovery requires both a valid raw sample and a filtered value >= 7.0 V.
  fakeAnalogValue = 860;
  assert(monitor.update(400));
  assert(monitor.voltageValid());
  assert(monitor.travelInhibited()); // Filter still below 7.0 V.
  assert(monitor.update(500));
  assert(!monitor.travelInhibited());

  // Low voltage trips immediately on the raw sample, without waiting for the
  // filtered value to decay below 6.8 V.
  fakeAnalogValue = 665; // Approximately 6.5 V at the pack.
  assert(monitor.update(600));
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());

  // An invalid high sample must not clear the low-battery latch.
  fakeAnalogValue = 1023;
  assert(monitor.update(700));
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // A valid but sub-7.0 V reading also cannot clear the latch.
  fakeAnalogValue = 706; // Approximately 6.9 V at the pack.
  assert(monitor.update(800));
  assert(monitor.voltageValid());
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());

  // Both raw and filtered readings eventually reach the recovery threshold.
  fakeAnalogValue = 860;
  assert(monitor.update(900));
  assert(!monitor.travelInhibited());
  return 0;
}
