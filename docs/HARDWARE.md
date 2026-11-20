# GyroGuard Hardware and Wiring

This guide is for a two-wheel differential-drive robot using an Arduino Uno R3 / classic Nano.

## 1. Mechanical parts

- 1 × Arduino Uno R3 or classic Nano (ATmega328P)
- 1 × MPU-6050 / GY-521 IMU module
- 2 × matched geared DC motors with quadrature encoders
- 2 × matched wheels (typical starting diameter: 65 mm)
- 1 × TB6612FNG dual motor driver breakout
- 1 × HC-SR04 ultrasonic sensor
- 1 × SG90 micro servo and rigid sensor bracket
- 1 × protected 2-cell Li-ion battery pack and compatible charger
- 1 × regulated 5 V buck converter with sufficient current for logic and servo
- Chassis, motor mounts, switch, wiring, fasteners, a fuse and a 10 kΩ STBY pull-down resistor

**Exactly two wheels:** mount one driven wheel on each side of the chassis, coaxial with one another. The robot balances dynamically on those two wheels. Do not add a caster during normal operation; a caster changes the dynamics and can hide balance problems.

## 2. Pin map

| Function | Arduino pin |
|---|---|
| MPU-6050 SDA | A4 |
| MPU-6050 SCL | A5 |
| TB6612 PWMA / left PWM | D5 |
| TB6612 AIN1 / left direction | D8 |
| TB6612 AIN2 / left direction | D10 |
| TB6612 PWMB / right PWM | D6 |
| TB6612 BIN1 / right direction | D11 |
| TB6612 BIN2 / right direction | D12 |
| TB6612 STBY | A3 |
| Left encoder channel A | D2 (INT0) |
| Left encoder channel B | D4 |
| Right encoder channel A | D3 (INT1) |
| Right encoder channel B | D7 |
| HC-SR04 TRIG | A0 |
| HC-SR04 ECHO | A1 (PCINT9) |
| SG90 signal | D9 |

Do not connect the HC-SR04 Echo to a different pin without changing the pin-change interrupt code. The Servo library reserves Timer1 on the Uno/Nano and disables hardware PWM on D9/D10 while attached; GyroGuard keeps motor PWM on D5/D6.

## 3. Power wiring

1. Battery positive goes through a correctly rated fuse and physical switch.
2. Feed the motor driver's VM from the motor battery only if the battery voltage is within the motor and driver breakout ratings.
3. Feed the buck converter input from the battery. Adjust and measure its output to **5.0 V before connecting electronics**.
4. Feed regulated 5 V to the Uno 5V pin, MPU-6050 VCC (only if your breakout supports 5 V input), HC-SR04 VCC and servo V+. The Uno must not also be powered from USB and an external 5 V rail at the same time unless the board's power arrangement explicitly supports it. During programming, disconnect the external 5 V feed or use a safe power-isolation method.
5. Connect all grounds together: battery negative, buck ground, Uno GND, TB6612 GND, sensor grounds and servo ground.
6. Do **not** power motors or the servo from the Uno 5 V pin. Servo current spikes can reset the controller; use a suitable 5 V buck rail with adequate peak-current capacity.
7. Keep motor wires twisted and physically separated from IMU and I2C wires. Add local bulk capacitance near the motor driver as recommended by its datasheet / breakout documentation.
8. Connect a **10 kΩ pull-down resistor from TB6612 STBY (A3) to GND**, unless the exact breakout already has a verified pull-down. Inspect the module for any STBY pull-up before adding the resistor.

Never connect a raw 2S battery directly to the Uno 5V pin. Do not connect a battery to Uno VIN unless the voltage is suitable for the board regulator; a 2S pack may not provide enough headroom for a stable 5 V rail.

## 4. TB6612FNG connections and current limits

- VCC → regulated 5 V logic
- VM → motor battery positive
- GND → common ground
- AO1/AO2 → left motor terminals
- BO1/BO2 → right motor terminals
- PWMA, AIN1, AIN2, PWMB, BIN1, BIN2 and STBY → pins in the table

Confirm the breakout's pin labels; board layouts differ.

**Do not size the motors using the driver's peak-current headline.** Toshiba's TB6612FNG datasheet lists an operating-range output-current limit of 1.0 A when VM is at least 4.5 V. Its absolute-maximum table lists 1.2 A per channel, with pulse-only limits of 2 A for a 20 ms pulse at no more than 20% duty and 3.2 A for a single 10 ms pulse. These absolute-maximum and pulse values are not continuous operating targets.

Before buying motors, check each motor's stall current at the intended motor voltage. If a motor can demand more current than the actual driver breakout can safely handle, choose a lower-current motor or a higher-current driver. Also account for both channels operating together, PCB copper area, cooling, wiring and battery capability; a breakout board can have a lower practical continuous-current limit than the IC headline rating.

Toshiba's typical application circuit shows local 0.1 µF and 10 µF decoupling on both the motor and logic supply rails. Check whether the breakout already includes suitable capacitors; otherwise add the recommended local decoupling close to the driver.

Reference: [Toshiba TB6612FNG official datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG).

## 5. MPU-6050 mounting

Mount the IMU rigidly on the chassis, close to the axle and away from motor vibration. For the default code orientation:

- PCB lies flat (components facing up).
- Sensor +X points toward the robot's front.
- Sensor +Y points toward the robot's left.
- Sensor +Z points upward.
- The robot's forward/backward pitch is rotation around +Y.

If your breakout axes or mounting differ, the pitch axis and sign in imu_mpu6050.cpp must be adapted. Do not try to fix a wrong IMU axis by randomly changing PID gains.

## 6. Encoder wiring and calibration

Each encoder needs channel A and channel B. Connect encoder supply and ground according to the motor/encoder datasheet. Verify encoder output voltage is compatible with 5 V Arduino inputs; use level shifting if required.

The code interrupts on channel A CHANGE and samples channel B (x2 decoding). Set ENCODER_COUNTS_PER_WHEEL_REV to:

    channel-A cycles per motor-shaft revolution × gearbox ratio × 2

Use the encoder manufacturer's definition carefully: some vendors quote counts after quadrature decoding or after the gearbox. If so, convert to the x2 convention used here. Alternatively, raise the wheels and count reported encoder ticks for exactly one wheel revolution.

If forward wheel rotation gives negative speed, change the corresponding LEFT_ENCODER_SIGN or RIGHT_ENCODER_SIGN in config.h.

## 7. Ultrasonic sensor and servo

Mount the HC-SR04 at the front, with a clear view and no frame parts in its beam. Attach it to the SG90 bracket. Verify the servo's physical left/right directions. If they are reversed, swap SERVO_LEFT_DEG and SERVO_RIGHT_DEG in config.h.

The sensor is scanned at center during travel, then left and right after an obstacle is detected. Echo capture is interrupt-driven; the balance loop is not blocked by pulseIn().

## 8. Battery voltage monitor (required for autonomous travel)

The firmware expects a protected **2S Li-ion pack** and a divider on A2. This monitor is a low-voltage travel inhibit; it is not a battery-management system and does not replace a protected pack or its BMS.

Wire the divider after the main fuse and power switch:

1. Connect switched battery positive to a **10 kΩ, 1% resistor**.
2. Connect the other end of that resistor to Arduino **A2**.
3. Connect a second **10 kΩ, 1% resistor** from A2 to common GND.
4. Connect a **100 nF ceramic capacitor** from A2 to GND, close to the controller.
5. Keep battery negative, divider ground, buck ground and Arduino GND common.

The equal-resistor divider has a 2:1 ratio. A fully charged 2S pack at 8.4 V produces approximately 4.2 V at A2. **Never allow A2 to exceed the Arduino's 5 V input limit.** Do not use this divider with a 3S pack.

Before connecting A2 to the controller, measure the divider output with a multimeter at the pack's maximum voltage. Calibrate `BATTERY_ADC_REFERENCE_MV` in config.h against the actual 5 V rail if the displayed pack voltage is inaccurate. The default 5000 mV reference is an initial value, not a precision measurement.

The firmware samples the pack every 100 ms, filters the result, and inhibits autonomous travel at or below 6.8 V. Once inhibited, travel remains blocked until the reading reaches 7.0 V. An invalid or missing divider reading also inhibits autonomous travel. The balance loop remains active; use the physical battery switch for an emergency stop. Do not deliberately deep-discharge the pack to test the threshold.

## 9. Before applying power

- Verify polarity and continuity with a multimeter.
- Check there is no short between battery positive and ground.
- Confirm the motor driver and motors are rated for the battery.
- Confirm all grounds are common.
- Verify the STBY pull-down and local driver decoupling are installed.
- Raise the wheels off the bench for the first motor test.
- Keep a hand on the physical power switch.
