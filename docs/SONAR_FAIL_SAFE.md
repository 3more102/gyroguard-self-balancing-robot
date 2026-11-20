# Ultrasonic Fail-Safe Navigation

The HC-SR04 is a limited-range sensor. A missing echo is an unknown measurement, not proof that the path is clear. GyroGuard therefore separates balance control from autonomous travel safety.

## Forward ranging

- One invalid forward measurement is tolerated while the next measurement is requested.
- Two consecutive invalid forward measurements enter SENSOR_HOLD.
- SENSOR_HOLD requests zero forward speed. The inner balance controller remains active.
- The controller retries forward ranging at the configured sonar retry interval.
- A valid reading above the obstacle stop threshold returns the navigation state to DRIVE.
- A valid reading inside the obstacle stop threshold starts the left/right scan.

## Side scanning

Each side measurement carries its own validity flag.

- If both sides are valid and one side meets the clearance threshold, the robot turns toward the side with the greater measured clearance.
- If exactly one side is valid and clear, the robot turns toward that verified side.
- If both sides are valid but neither meets the clearance threshold, the robot performs the configured reverse maneuver before turning toward the side with greater measured clearance.
- If neither side provides a verified clear path, the robot enters NO_PATH and requests zero travel speed.
- NO_PATH repeats the side scan after the configured rescan interval. It does not guess a direction from missing echoes.

## Commissioning

Keep autonomous mode disabled until balance-only operation is stable. Test the HC-SR04 with flat obstacles at known distances, then deliberately disconnect the sensor during a supported bench test to confirm that autonomous travel enters SENSOR_HOLD. Verify that the balance loop remains active and that travel resumes only after a valid forward measurement is received.

The fail-safe behavior does not replace a physical power switch, fuse, current-limited bench supply, tether, or supervised test area.
