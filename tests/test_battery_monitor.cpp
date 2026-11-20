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

  // 860 ADC counts with a 5 V reference and 2:1 divider is about 8.4 V.
  assert(monitor.update(0));
  assert(monitor.ready());
  assert(monitor.voltageValid());
  assert(!monitor.travelInhibited());
  assert(monitor.millivolts() == 8407U);

  // Sampling is rate-limited to 100 ms.
  fakeAnalogValue = 0;
  assert(!monitor.update(99));
  assert(!monitor.travelInhibited());

  // A raw out-of-range sample fails closed immediately, even if the filtered
  // voltage still looks plausible.
  fakeAnalogValue = 1023;
  assert(monitor.update(100));
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // A valid sample at the recovery threshold restores the voltage sense.
  fakeAnalogValue = 860;
  assert(monitor.update(200));
  assert(monitor.voltageValid());
  assert(!monitor.travelInhibited());

  // A disconnected divider is invalid and inhibits travel immediately.
  fakeAnalogValue = 0;
  assert(monitor.update(300));
  assert(!monitor.voltageValid());
  assert(monitor.travelInhibited());

  // A valid high reading restores travel; a raw low sample trips immediately.
  fakeAnalogValue = 860;
  assert(monitor.update(400));
  assert(monitor.voltageValid());
  assert(!monitor.travelInhibited());

  fakeAnalogValue = 695; // Approximately 6.8 V: at/below inhibit threshold.
  assert(monitor.update(500));
  assert(monitor.voltageValid());
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());

  // Hysteresis: 6.8–7.0 V remains inhibited.
  fakeAnalogValue = 716; // Just below 7.0 V.
  assert(monitor.update(600));
  assert(monitor.lowBattery());
  assert(monitor.travelInhibited());

  // At/above 7.0 V the low-battery latch clears.
  fakeAnalogValue = 717;
  assert(monitor.update(700));
  assert(!monitor.lowBattery());
  assert(!monitor.travelInhibited());
  return 0;
}
