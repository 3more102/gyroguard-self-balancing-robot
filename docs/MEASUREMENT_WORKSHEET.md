# GyroGuard — Hardware Parameter Measurement Worksheet

Complete this worksheet after selecting and assembling the actual hardware. The MATLAB model currently uses illustrative defaults; those defaults must not be presented as measured robot properties.

## 1. Required mechanical measurements

| Parameter | Measurement method | Unit | Recorded value |
|---|---|---:|---|
| Body mass above axle, excluding wheels | Weigh the assembled body, including battery, electronics, motor stators and brackets | kg | |
| Mass of one wheel | Weigh one complete wheel | kg | |
| Effective wheel radius | Measure axle-centre height above the floor with the normal load on the wheel | m | |
| Body centre-of-mass height | Locate the assembled body's centre of mass relative to the axle | m | |
| Body pitch inertia about its centre of mass | Obtain from CAD mass properties or a documented pendulum experiment | kg m² | |
| One-wheel rotational inertia | Obtain from CAD mass properties or estimate from the measured wheel geometry | kg m² | |
| Motor rotor inertia | Use the motor datasheet; otherwise record that it is unknown and identify the model experimentally | kg m² | |

For a uniform solid-disk approximation only, wheel inertia is \(J_w=\frac{1}{2}m_wr^2\). Use measured CAD geometry or a better physical model when the wheel is not close to a solid disk.

## 2. Required motor and drivetrain data

| Parameter | Measurement / source | Unit | Recorded value |
|---|---|---:|---|
| Gear ratio \(G\) | Motor/gearbox datasheet; define it as motor speed divided by wheel speed | dimensionless | |
| Gearbox efficiency \(\eta\) | Datasheet or documented identification estimate | dimensionless | |
| Winding resistance \(R\) | Measure motor terminal resistance; account for lead and brush variation | Ω | |
| Torque constant \(K_t\) | Motor datasheet; in SI units it is numerically equal to \(K_e\) for an ideal DC motor | N m/A | |
| Back-EMF constant \(K_e\) | Datasheet or measure generated voltage at a known motor-shaft speed | V s/rad | |
| Motor supply voltage under load | Measure at the driver VM terminals during a representative motor load | V | |
| Stall current | Motor datasheet; use it to verify driver and battery limits | A | |

Do not stall a motor for an extended period while measuring current. Use a current-limited supply and the motor manufacturer's permitted test duration.

## 3. Encoder and wheel calibration

1. Raise the chassis so both wheels are clear of the floor.
2. Mark each wheel and rotate it exactly one revolution by hand.
3. Record the encoder tick delta reported by firmware for each side.
4. The current firmware counts channel-A CHANGE edges and samples channel B (x2 decoding). Enter counts per wheel revolution using that convention in config.h.
5. Roll the robot forward by hand and confirm that both encoder counts and average speed are positive.
6. Measure the loaded wheel diameter and update WHEEL_DIAMETER_MM.

| Item | Left | Right |
|---|---:|---:|
| Counts per wheel revolution (firmware x2 convention) | | |
| Forward encoder sign (+1 / -1) | | |
| Motor forward sign (+1 / -1) | | |
| Loaded wheel diameter (mm) | | |

## 4. Damping and system identification

Mechanical translational damping and pitch damping are difficult to obtain accurately from a parts list. If they are not available from a reliable source:

- Record wheel coast-down speed after a controlled low-speed drive and estimate translational damping from the deceleration data.
- Record the body's small-angle free response with the drive disabled and estimate damping only after separating sensor filtering and mechanical effects.
- Prefer a documented least-squares/system-identification fit using measured input and response data over manually selecting values to make a simulation look stable.

Keep raw measurements, test conditions, battery voltage and uncertainty alongside every fitted parameter.

## 5. Sign-off before controller design

- [ ] All placeholder values in matlab/gyroguard_parameters.m have been replaced or explicitly justified.
- [ ] IMU orientation and pitch sign have been verified using docs/COMMISSIONING.md.
- [ ] Motor and encoder polarity have been verified with wheels raised.
- [ ] Motor stall current is within the actual driver-board and battery limits.
- [ ] The model's finite-difference linearization check passes.
- [ ] Model predictions have been compared with measured robot response.
- [ ] The final report distinguishes measured, datasheet, estimated and placeholder values.
