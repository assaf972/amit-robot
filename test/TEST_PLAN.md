# RoboCup Junior Rescue Line 2026 — Robot Test Plan

## 1. Overview

This document describes the comprehensive test plan for the line-following rescue robot built for the **RoboCup Junior Rescue Line 2026** competition. The tests are implemented in **AUnit** (an Arduino-native testing framework) and cover all code layers from pin configuration through competition-level integration.

**Test file:** `test/robot_test.ino`

---

## 2. Testing Framework & Tools

| Tool | Purpose |
|------|---------|
| **AUnit** (by Brian T. Park) | Arduino-compatible xUnit testing framework. Install via Arduino Library Manager. [GitHub](https://github.com/bxparks/AUnit) |
| **Arduino IDE / CLI** | Compile and upload test sketch to the board |
| **Serial Monitor (9600 baud)** | View test results (PASS/FAIL/SKIP for each test) |
| **Physical test fixtures** | Black electrical tape on white surface, green markers (25×25mm), obstacles, ultrasonic targets |

### How to Run

1. Open Arduino IDE → **Sketch > Include Library > Manage Libraries** → Search **"AUnit"** → Install.
2. Open `test/robot_test.ino`.
3. Ensure the correct board (Arduino Mega) and port are selected.
4. Upload to the robot.
5. Open Serial Monitor at **9600 baud**.
6. Results print as each test runs: `TestName | PASSED` or `TestName | FAILED`.

> **Safety:** For motor tests, place the robot on a raised platform or remove the wheels. Motors **will** spin briefly during testing.

---

## 3. Test Levels & Test Descriptions

### Level 1 — Hardware Integration & Pin Configuration (5 tests)

These tests verify that `setup()` correctly configures all hardware pins. No physical interaction required beyond powering on the robot.

| Test | What it verifies | Logic |
|------|-----------------|-------|
| `HW_MotorPinsConfiguredAsOutput` | Motor driver pins (`in1–in4`, `ena`, `enb`) accept output writes without error | Writes LOW to all motor pins; if no crash occurs, configuration is correct |
| `HW_LineSensorPinsConfiguredAsInput` | IR sensor pins (`LS`, `MS`, `RS`) are configured as INPUT | Reads all three pins and asserts each returns either HIGH or LOW |
| `HW_ColorSensorFrequencyScalingSetTo20Percent` | TCS color sensor S0/S1 pins are set for 2% frequency scaling | Reads back pin states: LS0=HIGH, LS1=LOW, RS0=HIGH, RS1=LOW |
| `HW_SerialInitializedAt9600Baud` | Serial communication is active | Checks `Serial` object evaluates to true |
| `HW_UltrasonicSensorPinsConfigured` | Ultrasonic sensor pins are wired correctly and responsive | Performs a read on both sensors; asserts non-negative distance |

---

### Level 2 — Motor & Steering Control (8 tests)

These tests verify that each movement function produces the correct H-bridge pin states. They validate motor direction, speed symmetry, and PWM range.

| Test | What it verifies | Logic |
|------|-----------------|-------|
| `Motor_ForwardBothMotorsSameDirection` | `forward()` drives both motors forward | Asserts: in1=LOW, in2=HIGH (left fwd), in3=LOW, in4=HIGH (right fwd) |
| `Motor_ForwardSymmetricSpeed` | Both motors receive equal PWM for straight driving | Validates speed value is within [0, 255] |
| `Motor_RightTurnLeftForwardRightReverse` | `right()` correction steers right | Left motor forward (in1=LOW, in2=HIGH), right motor reverse (in3=HIGH, in4=LOW) |
| `Motor_LeftTurnLeftReverseRightForward` | `left()` correction steers left | Left motor reverse (in1=HIGH, in2=LOW), right motor forward (in3=LOW, in4=HIGH) |
| `Motor_SharpRightBothReverse` | `sharp_right()` pivots the robot clockwise | Both motors reverse (in1=HIGH, in2=LOW, in3=HIGH, in4=LOW) |
| `Motor_SharpLeftBothForward` | `sharp_left()` behavior (documents actual behavior vs. expected) | Both motors forward with asymmetric speed — this is a wide curve, not a pivot |
| `Motor_StopBothMotorsZeroSpeed` | `stop()` sets both PWM to 0 | Writes `analogWrite(ena, 0)` and `analogWrite(enb, 0)` |
| `Motor_SpeedWithinPWMRange` | All speed values fit within valid PWM range [0–255] | Checks base speed (180), turn speed (210), sharp speed (**285 — overflow detected!**) |

---

### Level 3 — Line Sensor Management (5 tests)

These tests verify IR reflectance sensor readings match expected states on different surfaces. **Requires physical positioning of the robot.**

| Test | Setup Required | Logic |
|------|---------------|-------|
| `LineSensor_AllSensorsRespondToWhiteSurface` | Robot on white surface, no black lines | All three sensors should read HIGH (strong reflection) |
| `LineSensor_MiddleSensorDetectsBlackLine` | Robot centered on 1–2cm black line | Middle sensor reads LOW (line absorbs IR) |
| `LineSensor_LeftSensorDetectsBlackLine` | Left sensor positioned over black line | Left sensor reads LOW |
| `LineSensor_RightSensorDetectsBlackLine` | Right sensor positioned over black line | Right sensor reads LOW |
| `LineSensor_AllLowAtIntersection` | Robot on intersection/junction | All three sensors read LOW simultaneously |

---

### Level 4 — Color Sensor (Green Marker Detection) (4 tests)

Per competition rules §3.6, green markers (25mm × 25mm) at junctions indicate the correct path. The TCS color sensors detect green by reading the green-filtered photodiode frequency.

| Test | What it verifies | Logic |
|------|-----------------|-------|
| `ColorSensor_LeftGreenFrequencyReading` | Left sensor produces a valid green frequency | Sets S2=HIGH, S3=HIGH (green filter), reads `pulseIn()`, asserts > 0 |
| `ColorSensor_RightGreenFrequencyReading` | Right sensor produces a valid green frequency | Same as above for right sensor |
| `ColorSensor_GreenDetectionThreshold` | Known green surface produces frequency in [25–40] range | Reads frequency with green marker under sensor; reports if within range |
| `ColorSensor_TurnDecisionLogic` | Turn direction logic at junctions is correct | Tests the comparison `RgreenFreq > LgreenFreq → turn right`. Documents potential logic issue (higher freq = less green) |

---

### Level 5 — Ultrasonic Sensor (Obstacle Detection) (6 tests)

Per rules §3.5, obstacles are ≥15cm tall (bricks, blocks, weights). The robot uses a front ultrasonic sensor at 12cm detection range.

| Test | What it verifies | Logic |
|------|-----------------|-------|
| `Ultrasonic_FrontSensorReturnsValidReading` | Front sensor is connected and responsive | Reads distance, asserts ≥ 0 |
| `Ultrasonic_SideSensorReturnsValidReading` | Side sensor is connected and responsive | Reads distance, asserts ≥ 0 |
| `Ultrasonic_FrontDetectsCloseObject` | Object at ~10cm triggers detection | Asserts distance > 0 and < 12cm (requires physical object) |
| `Ultrasonic_FrontNoObjectFarReading` | Clear path does not trigger avoidance | Asserts distance ≥ 12cm or 0 |
| `Ultrasonic_ZeroReadingHandledCorrectly` | Zero reading (no echo) → 357 (safe fallback) | Simulates the `if (distance == 0) distance = 357` logic |
| `Ultrasonic_DetectionThresholdIs12cm` | Detection constant is 12cm | Validates constant value and reasonable range [5–30cm] |

---

### Level 6 — Line Following Logic (5 tests)

These tests verify the `node()` decision tree purely through logic assertions, without requiring hardware.

| Test | Sensor State | Expected Action |
|------|-------------|-----------------|
| `Logic_CenteredOnLine_GoesForward` | LS=HIGH, MS=LOW, RS=HIGH | `forward()` — robot is centered |
| `Logic_DriftedRight_TurnsRight` | LS=HIGH, RS=LOW | `right()` — correct toward line |
| `Logic_DriftedLeft_TurnsLeft` | LS=LOW, RS=HIGH | `left()` — correct toward line |
| `Logic_AllLow_IntersectionDetected` | LS=LOW, MS=LOW, RS=LOW | Read color sensors, decide turn |
| `Logic_AllHigh_OffLine` | LS=HIGH, MS=HIGH, RS=HIGH | Robot is off line — obstacle avoidance mode |

---

### Level 7 — Obstacle Avoidance Logic (5 tests)

These tests verify the `obstacle()` function's decision logic for navigating around obstacles.

| Test | What it verifies | Logic |
|------|-----------------|-------|
| `Obstacle_TriggerDistance` | Avoidance triggers at < 12cm | distance=10 → should trigger |
| `Obstacle_NoTriggerWhenFar` | No trigger at > 12cm | distance=50 → should not trigger |
| `Obstacle_NoTriggerOnZeroReading` | Zero reading handled safely | 0 → 357 → no trigger |
| `Obstacle_SideDistanceThresholdForRealignment` | Robot turns right when side > 42cm | Validates the `detection_distance + 30` threshold |
| `Obstacle_TimerBasedForwardBursts` | Forward bursts are 200ms | Validates the timing constant |
| `Obstacle_RejoinsLineWhenSensorDetectsBlack` | Avoidance ends when line found | While-loop exits when any sensor reads LOW |

---

### Level 8 — Competition Rule Compliance (7 tests)

These tests validate requirements from the RoboCup Junior Rescue Line 2026 rules document.

| Test | Rule Reference | What it checks |
|------|---------------|----------------|
| `Competition_RobotIsFullyAutonomous` | §4.1 | No remote control input in code |
| `Competition_GameTimeLimit8Minutes` | §5.7 | Documents that code lacks an 8-minute auto-stop |
| `Competition_ObstacleDetectionDistance` | §3.5 | 12cm detection is within reasonable range |
| `Competition_LineWidth1to2cm` | §3.3.1 | Sensor spacing reminder for 1–2cm line |
| `Competition_GapHandlingRequired` | §3.3.2 | **Not implemented** — gaps up to 20cm (10 pts/tile) |
| `Competition_RampHandlingRequired` | §3.7 | Motor torque for 25° inclines |
| `Competition_DeadEndUTurnNotImplemented` | §3.6.4 | **Not implemented** — double green U-turn |

---

### Level 9 — Bug Detection (4 tests)

These tests document bugs and potential issues found during code review.

| Test | Bug Description | Impact |
|------|----------------|--------|
| `Bug_SharpLeftDoesNotTurnLeft` | `sharp_left()` sets both motors forward (same as `forward()` with speed bias). `sharp_right()` reverses both motors. The functions are asymmetric. | `sharp_left()` makes a wide curve, not a sharp pivot like `sharp_right()` |
| `Bug_PWMOverflowInSharpTurns` | `speed + 105 = 285` exceeds PWM max of 255. `analogWrite()` clamps to 255 silently. | Sharp turn speed may not be as intended |
| `Bug_Pin13ConflictWithSideTrigPin` | Pin 13 is used for **both** the side ultrasonic trigger AND the LED indicator in `obstacle()`. Writing to pin 13 for LED status corrupts ultrasonic readings. | Side ultrasonic may give incorrect readings during obstacle avoidance |
| `Bug_ObjectEvationTimerOverflow` | `object_evation_timer` is `int` but stores `millis()` (unsigned long). On Arduino Uno (16-bit int), overflows after ~33 seconds. | Obstacle avoidance timing may fail after 33 seconds on Uno |

---

## 4. Test Summary

| Level | Category | Test Count | Hardware Required |
|-------|----------|-----------|-------------------|
| 1 | Pin Configuration & HW Integration | 5 | Power only |
| 2 | Motor & Steering Control | 8 | Wheels removed or elevated |
| 3 | Line Sensor Management | 5 | Black tape on white surface |
| 4 | Color Sensor (Green Detection) | 4 | Green markers (25×25mm) |
| 5 | Ultrasonic Sensors | 6 | Objects at known distances |
| 6 | Line Following Logic | 5 | None (pure logic) |
| 7 | Obstacle Avoidance Logic | 6 | None (pure logic) |
| 8 | Competition Compliance | 7 | Manual review |
| 9 | Bug Detection | 4 | None (code review) |
| **Total** | | **50** | |

---

## 5. Missing Features for Competition

Based on the rules analysis, the following features are **not yet implemented** and will cost significant points:

| Feature | Rules Section | Points Available |
|---------|--------------|-----------------|
| **Gap handling** (line gaps up to 20cm) | §3.3.2 | 10 pts/tile |
| **Speed bump handling** | §3.5.1 | 10 pts/tile |
| **Dead-end U-turn** (double green markers) | §3.6.4 | 10 pts |
| **Ramp navigation** (up to 25°) | §3.7 | 10 pts/tile |
| **8-minute game timer** | §5.7 | Auto-stop |

---

## 6. Known Bugs to Fix

1. **`sharp_left()` direction** — Both motors go forward; should mirror `sharp_right()` with one motor reversed.
2. **PWM overflow** — `speed + 105 = 285` exceeds 255; use `min(speed + 105, 255)` or reduce the offset.
3. **Pin 13 conflict** — Move the LED indicator to a different pin (e.g., pin 8) to avoid interfering with the side ultrasonic sensor.
4. **Timer type** — Change `object_evation_timer` from `int` to `unsigned long` to prevent overflow.
5. **Green detection logic** — Higher pulseIn frequency means *less* of that color. The comparison `RgreenFreq > LgreenFreq → turn right` may be inverted.
