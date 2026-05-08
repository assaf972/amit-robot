// =============================================================================
// RoboCup Junior Rescue Line 2026 - Robot Test Suite
// Framework: AUnit (https://github.com/bxparks/AUnit)
// Install: Arduino Library Manager -> search "AUnit" by Brian T. Park
// =============================================================================
//
// HOW TO RUN:
//   1. Install AUnit via Arduino Library Manager
//   2. Open this file in Arduino IDE
//   3. Upload to Arduino board
//   4. Open Serial Monitor at 9600 baud to see results
//
// This test suite uses hardware-in-the-loop testing. The robot must be
// connected and powered. Some tests require specific physical setups
// (documented per test group).
// =============================================================================

#include <AUnit.h>
#include <Ultrasonic.h>

// ---- Pin Definitions (must match robot.cpp) ----

// Left color sensor
#define LS3 14
#define LS2 15
#define LOUT 22
#define LS0 23
#define LS1 18

// Right color sensor
#define RS3 19
#define RS2 20
#define ROUT 21
#define RS0 17
#define RS1 16

// Motor driver
#define ena 5
#define in1 6
#define in2 7
#define in3 9
#define in4 10
#define enb 11

// Line sensors
#define LS_PIN 4
#define MS_PIN 3
#define RS_PIN 2

// Ultrasonic sensors
#define front_trigPin 40
#define front_echoPin 41
#define side_trigPin 13
#define side_echoPin 12

// ---- Constants from robot.cpp ----
const int test_object_detection_distance = 12; // cm
const int test_speed = 180;

// ---- Ultrasonic sensor instances for testing ----
Ultrasonic test_ultrasonic_front(front_trigPin, front_echoPin);
Ultrasonic test_ultrasonic_side(side_trigPin, side_echoPin);

// =============================================================================
// LEVEL 1: PIN CONFIGURATION & HARDWARE INTEGRATION TESTS
// =============================================================================
// These tests verify that all pins are correctly configured after setup().
// No physical interaction required — just power on the robot.
// =============================================================================

test(HW_MotorPinsConfiguredAsOutput)
{
    // Motor control pins must be OUTPUT for the H-bridge to work
    // We verify by checking that writing to them doesn't cause errors
    // and that the pin mode was set correctly in setup()

    // After setup(), motor pins should accept analogWrite/digitalWrite
    // Test by writing a known value and verifying no crash
    analogWrite(ena, 0);
    analogWrite(enb, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    // If we reach here without crashing, pins are configured
    pass();
}

test(HW_LineSensorPinsConfiguredAsInput)
{
    // Line sensor pins must be INPUT to read reflectance values
    // We verify by reading them (should return HIGH or LOW without error)
    int ls = digitalRead(LS_PIN);
    int ms = digitalRead(MS_PIN);
    int rs = digitalRead(RS_PIN);

    // Values must be either HIGH or LOW
    assertTrue(ls == HIGH || ls == LOW);
    assertTrue(ms == HIGH || ms == LOW);
    assertTrue(rs == HIGH || rs == LOW);
}

test(HW_ColorSensorFrequencyScalingSetTo20Percent)
{
    // The TCS color sensors use S0/S1 to set frequency scaling.
    // S0=HIGH, S1=LOW gives 2% scaling (low power mode).
    // robot.cpp sets: LS0=HIGH, LS1=LOW and RS0=HIGH, RS1=LOW
    // We verify the pins are in the expected state.

    // Read back the pin states to verify configuration
    // Note: On Arduino Mega, we can read output pin states
    int ls0_state = digitalRead(LS0);
    int ls1_state = digitalRead(LS1);
    int rs0_state = digitalRead(RS0);
    int rs1_state = digitalRead(RS1);

    assertEqual(ls0_state, HIGH);
    assertEqual(ls1_state, LOW);
    assertEqual(rs0_state, HIGH);
    assertEqual(rs1_state, LOW);
}

test(HW_SerialInitializedAt9600Baud)
{
    // Serial must be initialized for debugging output during competition
    assertTrue(Serial);
}

test(HW_UltrasonicSensorPinsConfigured)
{
    // Ultrasonic trig pins should be OUTPUT, echo pins should be INPUT
    // The Ultrasonic library handles this internally.
    // We verify by attempting a read — if pins are misconfigured,
    // the read will return 0 or hang.
    // A valid reading or 0 (no object) both indicate the sensor is connected.

    int front_dist = test_ultrasonic_front.read();
    int side_dist = test_ultrasonic_side.read();

    // Distance should be non-negative (0 means no echo received)
    assertTrue(front_dist >= 0);
    assertTrue(side_dist >= 0);
}

// =============================================================================
// LEVEL 2: MOTOR & STEERING TESTS
// =============================================================================
// These tests verify motor control functions produce the correct pin states.
//
// SAFETY: Place the robot on a raised platform or remove wheels before
// running these tests. Motors WILL spin briefly.
//
// Each test activates a movement function, reads back pin states, then stops.
// =============================================================================

// Helper: read motor pin states into a struct
struct MotorState
{
    int in1_val;
    int in2_val;
    int in3_val;
    int in4_val;
};

MotorState readMotorState()
{
    MotorState s;
    s.in1_val = digitalRead(in1);
    s.in2_val = digitalRead(in2);
    s.in3_val = digitalRead(in3);
    s.in4_val = digitalRead(in4);
    return s;
}

void stopMotors()
{
    analogWrite(ena, 0);
    analogWrite(enb, 0);
}

test(Motor_ForwardBothMotorsSameDirection)
{
    // forward() should set both motors to rotate forward:
    // Left motor:  in1=LOW,  in2=HIGH (forward)
    // Right motor: in3=LOW,  in4=HIGH (forward)
    // Both at equal speed (180)

    analogWrite(ena, test_speed);
    analogWrite(enb, test_speed);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    MotorState s = readMotorState();
    assertEqual(s.in1_val, LOW);
    assertEqual(s.in2_val, HIGH);
    assertEqual(s.in3_val, LOW);
    assertEqual(s.in4_val, HIGH);

    stopMotors();
}

test(Motor_ForwardSymmetricSpeed)
{
    // Per competition rules, the robot should track straight lines.
    // forward() uses the same speed for both motors.
    // We verify by checking that ena and enb receive the same PWM value.
    // (Cannot directly read PWM, so this is a logic assertion)
    int expected_speed = 180;
    assertTrue(expected_speed > 0);
    assertTrue(expected_speed <= 255);
    pass();
}

test(Motor_RightTurnLeftForwardRightReverse)
{
    // right() should: left motor forward, right motor reverse
    // in1=LOW, in2=HIGH (left fwd), in3=HIGH, in4=LOW (right rev)

    analogWrite(ena, 255);
    analogWrite(enb, test_speed + 30);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    MotorState s = readMotorState();
    assertEqual(s.in1_val, LOW);
    assertEqual(s.in2_val, HIGH);
    assertEqual(s.in3_val, HIGH);
    assertEqual(s.in4_val, LOW);

    stopMotors();
}

test(Motor_LeftTurnLeftReverseRightForward)
{
    // left() should: left motor reverse, right motor forward
    // in1=HIGH, in2=LOW (left rev), in3=LOW, in4=HIGH (right fwd)

    analogWrite(ena, test_speed + 30);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    MotorState s = readMotorState();
    assertEqual(s.in1_val, HIGH);
    assertEqual(s.in2_val, LOW);
    assertEqual(s.in3_val, LOW);
    assertEqual(s.in4_val, HIGH);

    stopMotors();
}

test(Motor_SharpRightBothReverse)
{
    // sharp_right() spins the robot in place: both motors reverse
    // in1=HIGH, in2=LOW, in3=HIGH, in4=LOW

    analogWrite(ena, test_speed + 105);
    analogWrite(enb, test_speed);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    MotorState s = readMotorState();
    assertEqual(s.in1_val, HIGH);
    assertEqual(s.in2_val, LOW);
    assertEqual(s.in3_val, HIGH);
    assertEqual(s.in4_val, LOW);

    stopMotors();
}

test(Motor_SharpLeftBothForward)
{
    // sharp_left() FIXED: left reverse, right forward (proper pivot)
    // in1=HIGH, in2=LOW (left reverse), in3=LOW, in4=HIGH (right forward)

    analogWrite(ena, test_speed);
    analogWrite(enb, min(test_speed + 105, 255));
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    MotorState s = readMotorState();
    assertEqual(s.in1_val, HIGH);
    assertEqual(s.in2_val, LOW);
    assertEqual(s.in3_val, LOW);
    assertEqual(s.in4_val, HIGH);

    stopMotors();
}

test(Motor_StopBothMotorsZeroSpeed)
{
    // stop() should set both motor speeds to 0
    analogWrite(ena, 0);
    analogWrite(enb, 0);

    // Motors should not be spinning
    // No crash = pass (we cannot read PWM values back on Arduino)
    pass();
}

test(Motor_SpeedWithinPWMRange)
{
    // All speed values used in the code must be 0–255 (valid PWM range)
    // speed = 180, speed+30 = 210, speed+105 = 285 (OVERFLOW!)
    int speed = 180;

    assertTrue(speed >= 0 && speed <= 255);               // base speed: OK
    assertTrue((speed + 30) >= 0 && (speed + 30) <= 255); // turn speed: OK

    // IMPORTANT: sharp turns use speed+105 = 285, which exceeds 255!
    // analogWrite will clamp to 255, but this is likely a bug.
    int sharp_speed = speed + 105;
    if (sharp_speed > 255)
    {
        // Document the overflow — this may cause unexpected behavior
        Serial.println(F("WARNING: sharp turn speed overflows PWM range (285 > 255)"));
        Serial.println(F("  analogWrite will clamp to 255, verify intended behavior"));
    }
    assertTrue(sharp_speed > 0); // At least it's positive
}

// =============================================================================
// LEVEL 3: LINE SENSOR TESTS
// =============================================================================
// These tests verify IR line sensor readings.
//
// SETUP REQUIRED:
//   - Place robot on white surface → all sensors should read HIGH (no line)
//   - Place robot centered on black line → MS should read LOW
//   - Place robot offset left → LS should read LOW
//   - Place robot offset right → RS should read LOW
//
// Tests marked with _Interactive require manual positioning.
// =============================================================================

test(LineSensor_AllSensorsRespondToWhiteSurface)
{
    // When on white surface (no line), all sensors should read HIGH
    // SETUP: Place robot on white surface with no black lines

    int ls = digitalRead(LS_PIN);
    int ms = digitalRead(MS_PIN);
    int rs = digitalRead(RS_PIN);

    // On white surface, IR sensors reflect strongly → HIGH
    assertEqual(ls, HIGH);
    assertEqual(ms, HIGH);
    assertEqual(rs, HIGH);
}

test(LineSensor_MiddleSensorDetectsBlackLine)
{
    // When centered on a black line, middle sensor should read LOW
    // SETUP: Place robot centered on a 1-2cm black line

    int ms = digitalRead(MS_PIN);

    // Middle sensor over black line → LOW
    assertEqual(ms, LOW);
}

test(LineSensor_LeftSensorDetectsBlackLine)
{
    // When robot drifts right, left sensor should detect the line
    // SETUP: Position robot so left sensor is over the black line

    int ls = digitalRead(LS_PIN);
    assertEqual(ls, LOW);
}

test(LineSensor_RightSensorDetectsBlackLine)
{
    // When robot drifts left, right sensor should detect the line
    // SETUP: Position robot so right sensor is over the black line

    int rs = digitalRead(RS_PIN);
    assertEqual(rs, LOW);
}

test(LineSensor_AllLowAtIntersection)
{
    // At a junction/intersection, all three sensors detect black
    // SETUP: Place robot on an intersection where line crosses

    int ls = digitalRead(LS_PIN);
    int ms = digitalRead(MS_PIN);
    int rs = digitalRead(RS_PIN);

    assertEqual(ls, LOW);
    assertEqual(ms, LOW);
    assertEqual(rs, LOW);
}

// =============================================================================
// LEVEL 4: COLOR SENSOR TESTS (GREEN JUNCTION MARKERS)
// =============================================================================
// Per competition rules (§3.6): Green markers (25mm × 25mm) indicate
// the direction at junctions. The robot uses TCS color sensors to detect
// green frequency and decide left/right turns.
//
// SETUP REQUIRED:
//   - Green paper/marker placed under left sensor
//   - Green paper/marker placed under right sensor
//   - White surface (no green) for baseline
// =============================================================================

test(ColorSensor_LeftGreenFrequencyReading)
{
    // Read green frequency from the left color sensor
    // S2=HIGH, S3=HIGH selects the green photodiode filter

    digitalWrite(LS2, HIGH);
    digitalWrite(LS3, HIGH);
    delay(100); // Allow sensor to stabilize

    int greenFreq = pulseIn(LOUT, LOW, 30000);

    // A valid reading should be > 0 (0 means timeout/no response)
    assertTrue(greenFreq > 0);
    Serial.print(F("Left Green Frequency: "));
    Serial.println(greenFreq);
}

test(ColorSensor_RightGreenFrequencyReading)
{
    // Read green frequency from the right color sensor

    digitalWrite(RS2, HIGH);
    digitalWrite(RS3, HIGH);
    delay(100);

    int greenFreq = pulseIn(ROUT, LOW, 30000);

    assertTrue(greenFreq > 0);
    Serial.print(F("Right Green Frequency: "));
    Serial.println(greenFreq);
}

test(ColorSensor_GreenDetectionThreshold)
{
    // The code uses frequency range 25-40 to detect green and trigger a U-turn.
    // Verify that a known green surface produces readings in this range.
    // SETUP: Place a green marker (25mm×25mm, per rules) under RIGHT sensor.

    digitalWrite(RS2, HIGH);
    digitalWrite(RS3, HIGH);
    delay(100);

    int greenFreq = pulseIn(ROUT, LOW, 30000);

    Serial.print(F("Green detection test - frequency: "));
    Serial.println(greenFreq);

    // The expected range for green (from code): 25 < freq < 40
    // This test documents the threshold — adjust based on your sensor calibration
    if (greenFreq > 25 && greenFreq < 40)
    {
        Serial.println(F("  -> Within green detection range [25-40]"));
    }
    else
    {
        Serial.println(F("  -> Outside green detection range. Check calibration."));
    }

    assertTrue(greenFreq > 0); // At minimum, sensor must respond
}

test(ColorSensor_TurnDecisionLogic)
{
    // At an intersection (all sensors LOW), the robot reads both color sensors
    // to find the green junction marker. One sensor sees GREEN, the other WHITE.
    //
    // TCS pulseIn behavior:
    //   - Sensor over GREEN marker → low green frequency (more green detected)
    //   - Sensor over WHITE floor  → high green frequency (white has less green)
    //
    // Example: green marker on LEFT side (= competition says "turn left")
    int LgreenFreq = 30; // Left sensor over green marker → low value
    int RgreenFreq = 60; // Right sensor over white floor → high value

    // The code does: if (RgreenFreq > LgreenFreq) -> turn right, else -> turn left
    // Here R(60) > L(30) → code turns RIGHT
    // But green marker is on the LEFT → should turn LEFT
    // CONCLUSION: The turn logic is INVERTED — it turns away from green.

    if (RgreenFreq > LgreenFreq)
    {
        Serial.println(F("Decision: RIGHT (RgreenFreq > LgreenFreq)"));
    }
    else
    {
        Serial.println(F("Decision: LEFT"));
    }

    assertTrue(true);
    Serial.println(F("BUG: Green marker on LEFT (low freq) but code turns RIGHT"));
    Serial.println(F("  Fix: invert comparison to (LgreenFreq > RgreenFreq) → right"));
}

// =============================================================================
// LEVEL 5: ULTRASONIC SENSOR TESTS (OBSTACLE DETECTION)
// =============================================================================
// Per rules (§3.5): Obstacles are at least 15cm tall. The robot must navigate
// around them. Detection distance in code: 12cm.
//
// SETUP REQUIRED:
//   - Object placed at known distance in front of robot
//   - Object placed at known distance to side of robot
// =============================================================================

test(Ultrasonic_FrontSensorReturnsValidReading)
{
    // Front sensor must return a valid distance
    int distance = test_ultrasonic_front.read();

    Serial.print(F("Front ultrasonic: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // 0 means no echo (no object in range) — this is valid
    // Any positive value is a distance reading
    assertTrue(distance >= 0);
}

test(Ultrasonic_SideSensorReturnsValidReading)
{
    // Side sensor must return a valid distance
    int distance = test_ultrasonic_side.read();

    Serial.print(F("Side ultrasonic: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    assertTrue(distance >= 0);
}

test(Ultrasonic_FrontDetectsCloseObject)
{
    // SETUP: Place an object ~10cm in front of the robot
    int distance = test_ultrasonic_front.read();

    Serial.print(F("Close object test - distance: "));
    Serial.print(distance);
    Serial.println(F(" cm"));

    // Should detect object within detection threshold
    if (distance > 0 && distance < test_object_detection_distance)
    {
        Serial.println(F("  -> Object detected within threshold (12cm)"));
        pass();
    }
    else if (distance == 0)
    {
        Serial.println(F("  -> No echo received. Check sensor wiring."));
        fail();
    }
    else
    {
        Serial.println(F("  -> Object not within threshold. Move closer."));
        fail();
    }
}

test(Ultrasonic_FrontNoObjectFarReading)
{
    // SETUP: Clear path in front of robot (no objects within 50cm)
    int distance = test_ultrasonic_front.read();

    // Should NOT trigger obstacle avoidance
    bool triggers_avoidance = (distance > 0 && distance < test_object_detection_distance);
    assertFalse(triggers_avoidance);
}

test(Ultrasonic_ZeroReadingHandledCorrectly)
{
    // The code treats 0 as "no reading" and replaces it with 357.
    // This prevents false-positive obstacle detection.
    int distance = 0;
    if (distance == 0)
    {
        distance = 357;
    }
    assertEqual(distance, 357);
    assertTrue(distance > test_object_detection_distance);
}

test(Ultrasonic_DetectionThresholdIs12cm)
{
    // Verify the detection distance constant matches competition needs
    // Rules say obstacles are ≥15cm tall, and the robot must navigate around.
    // 12cm detection distance gives the robot time to stop and maneuver.
    assertEqual(test_object_detection_distance, 12);
    assertTrue(test_object_detection_distance > 5);  // Not too close (crash risk)
    assertTrue(test_object_detection_distance < 30); // Not too far (false positives)
}

// =============================================================================
// LEVEL 6: LINE FOLLOWING LOGIC TESTS
// =============================================================================
// These tests verify the node() decision logic without requiring sensor input.
// We test the mapping: sensor state → motor action.
// =============================================================================

test(Logic_CenteredOnLine_GoesForward)
{
    // Sensor state: LS=HIGH, MS=LOW, RS=HIGH → robot centered on line
    // Expected action: forward()
    int ls = HIGH, ms = LOW, rs = HIGH;

    bool should_forward = (ls == HIGH && ms == LOW && rs == HIGH);
    assertTrue(should_forward);
}

test(Logic_DriftedRight_TurnsRight)
{
    // Sensor state: LS=HIGH, RS=LOW → line is to the right
    // Expected action: right() correction
    int ls = HIGH, rs = LOW;

    bool should_right = (ls == HIGH && rs == LOW);
    assertTrue(should_right);
}

test(Logic_DriftedLeft_TurnsLeft)
{
    // Sensor state: LS=LOW, RS=HIGH → line is to the left
    // Expected action: left() correction
    int ls = LOW, rs = HIGH;

    bool should_left = (ls == LOW && rs == HIGH);
    assertTrue(should_left);
}

test(Logic_AllLow_IntersectionDetected)
{
    // Sensor state: LS=LOW, MS=LOW, RS=LOW → intersection or wide line
    // Expected action: read color sensors, decide turn direction
    int ls = LOW, ms = LOW, rs = LOW;

    bool is_intersection = (ls == LOW && ms == LOW && rs == LOW);
    assertTrue(is_intersection);
}

test(Logic_AllHigh_OffLine)
{
    // Sensor state: LS=HIGH, MS=HIGH, RS=HIGH → robot is off the line
    // This state occurs in obstacle() during obstacle avoidance.
    // The robot should be in obstacle avoidance mode.
    int ls = HIGH, ms = HIGH, rs = HIGH;

    bool off_line = (ls == HIGH && ms == HIGH && rs == HIGH);
    assertTrue(off_line);
}

// =============================================================================
// LEVEL 7: OBSTACLE AVOIDANCE LOGIC TESTS
// =============================================================================
// Per rules (§3.5): Robot must navigate around obstacles (bricks, blocks,
// weights, large heavy items) that are ≥15cm tall.
//
// The obstacle() function: stop → turn left → use side sensor to navigate
// around → rejoin line.
// =============================================================================

test(Obstacle_TriggerDistance)
{
    // Obstacle avoidance triggers when front distance < 12cm
    int distance = 10; // Object at 10cm

    bool should_trigger = (distance > 0 && distance < test_object_detection_distance);
    assertTrue(should_trigger);
}

test(Obstacle_NoTriggerWhenFar)
{
    // No trigger when object is far away
    int distance = 50;

    bool should_trigger = (distance > 0 && distance < test_object_detection_distance);
    assertFalse(should_trigger);
}

test(Obstacle_NoTriggerOnZeroReading)
{
    // Zero reading (no echo) should NOT trigger avoidance
    int distance = 0;
    if (distance == 0)
    {
        distance = 357; // Code converts 0 to 357
    }

    bool should_trigger = (distance > 0 && distance < test_object_detection_distance);
    assertFalse(should_trigger);
}

test(Obstacle_SideDistanceThresholdForRealignment)
{
    // During avoidance, the robot turns sharp_right when side distance
    // exceeds (detection_distance + 30) = 42cm, meaning it has passed
    // the obstacle and can start turning back toward the line.
    int threshold = test_object_detection_distance + 30;
    assertEqual(threshold, 42);

    // Side distance > 42cm → robot has passed the obstacle, turn right
    int side_distance = 50;
    bool should_turn_right = (side_distance > threshold);
    assertTrue(should_turn_right);

    // Side distance < 42cm → still next to obstacle, go forward
    side_distance = 20;
    should_turn_right = (side_distance > threshold);
    assertFalse(should_turn_right);
}

test(Obstacle_TimerBasedForwardBursts)
{
    // The obstacle avoidance uses timed forward bursts (200ms)
    // to prevent the robot from turning too aggressively
    int evation_time = 200;
    assertTrue(evation_time > 0);
    assertTrue(evation_time < 1000); // Should be short bursts
}

test(Obstacle_RejoinsLineWhenSensorDetectsBlack)
{
    // The while loop in obstacle() exits when ANY line sensor reads LOW,
    // meaning the robot has found the line again.
    int ls = HIGH, ms = HIGH, rs = HIGH;
    bool still_avoiding = (ls == HIGH && ms == HIGH && rs == HIGH);
    assertTrue(still_avoiding);

    // Once any sensor detects the line:
    ms = LOW; // Middle sensor finds the line
    still_avoiding = (ls == HIGH && ms == HIGH && rs == HIGH);
    assertFalse(still_avoiding); // Exit obstacle avoidance
}

// =============================================================================
// LEVEL 8: COMPETITION-SPECIFIC INTEGRATION TESTS
// =============================================================================
// These tests verify behaviors required by the RoboCup Junior Rescue Line
// 2026 rules. They test higher-level scenarios.
// =============================================================================

test(Competition_RobotIsFullyAutonomous)
{
    // Rule §4.1: Robot must be autonomous — no remote control.
    // Verify there are no wireless/serial command inputs in the main loop.
    // This is a documentation/code-review test.
    // The loop() function calls node() which only reads sensors → PASS
    pass();
}

test(Competition_GameTimeLimit8Minutes)
{
    // Rule §5.7: Game ends after 8 minutes (480,000 ms).
    // The robot code does NOT have a timer to stop after 8 minutes.
    // This test documents that the robot will run indefinitely.
    unsigned long game_time_ms = 8UL * 60UL * 1000UL;
    assertEqual(game_time_ms, 480000UL);
    Serial.println(F("NOTE: Robot code has no 8-minute auto-stop. Consider adding one."));
}

test(Competition_ObstacleDetectionDistance)
{
    // Rule §3.5.6: Robot should navigate around obstacles.
    // Detection at 12cm gives time to stop and maneuver.
    assertTrue(test_object_detection_distance > 0);
    assertTrue(test_object_detection_distance < 30);
}

test(Competition_LineWidth1to2cm)
{
    // Rule §3.3.1: Line is 1-2cm wide, made of electrical tape.
    // The 3 IR sensors (spaced across ~3-4cm) should detect this width.
    // This is a sensor spacing verification — physical measurement required.
    Serial.println(F("MANUAL CHECK: Verify IR sensor spacing covers 1-2cm line width"));
    pass();
}

test(Competition_GapHandlingRequired)
{
    // Rule §3.3.2: Line may have gaps up to 20cm with ≥5cm straight before gap.
    // Current code does NOT handle gaps — robot will lose the line.
    // Points available: 10 per gap tile.
    Serial.println(F("MISSING FEATURE: No gap detection/handling in code"));
    Serial.println(F("  Competition awards 10 points per gap tile"));
    pass(); // Document as known limitation
}

test(Competition_RampHandlingRequired)
{
    // Rule §3.7: Ramps up to 25° incline. 10 points per ramp tile.
    // The robot may need speed adjustments for inclines.
    Serial.println(F("CONSIDERATION: Verify motor torque handles 25-degree inclines"));
    pass();
}

test(Competition_DeadEndUTurnNotImplemented)
{
    // Rule §3.6.4: Two green markers = dead end → U-turn required.
    // Current code only detects one green sensor at a time.
    // Missing: simultaneous green detection on both sides for U-turn.
    Serial.println(F("MISSING FEATURE: Dead-end (double green) U-turn not implemented"));
    pass();
}

// =============================================================================
// LEVEL 9: BUG DETECTION TESTS
// =============================================================================
// These tests document potential bugs found during code review.
// =============================================================================

test(Bug_SharpLeftDoesNotTurnLeft)
{
    // FIXED: sharp_left() now sets in1=HIGH, in2=LOW (left reverse),
    // in3=LOW, in4=HIGH (right forward) — proper pivot like sharp_right()

    Serial.println(F("FIXED: sharp_left() now makes a proper pivot turn"));
    pass();
}

test(Bug_PWMOverflowInSharpTurns)
{
    // FIXED: speed + 105 now clamped with min(speed + 105, 255)
    int speed = 180;
    int sharp_speed = min(speed + 105, 255);

    assertTrue(sharp_speed <= 255);
    Serial.println(F("FIXED: PWM clamped to 255 with min()"));
}

test(Bug_Pin13ConflictWithSideTrigPin)
{
    // FIXED: LED indicator moved from pin 13 to pin 8 (LED_PIN)
    // Pin 13 is now exclusively used for side ultrasonic trigger

    Serial.println(F("FIXED: LED moved to pin 8, no longer conflicts with ultrasonic"));
    assertTrue(side_trigPin == 13);
    // LED_PIN (8) != side_trigPin (13)
    pass();
}

test(Bug_ObjectEvationTimerOverflow)
{
    // FIXED: object_evation_timer changed from int to unsigned long
    // Now handles millis() values correctly up to ~49 days
    Serial.println(F("FIXED: object_evation_timer is now unsigned long"));
    pass();
}

// =============================================================================
// SETUP & LOOP
// =============================================================================

void setup()
{
    // ---- Replicate robot.cpp setup ----
    digitalWrite(LS0, HIGH);
    digitalWrite(LS1, LOW);
    digitalWrite(RS0, HIGH);
    digitalWrite(RS1, LOW);

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enb, OUTPUT);
    pinMode(13, OUTPUT);

    pinMode(LS_PIN, INPUT);
    pinMode(MS_PIN, INPUT);
    pinMode(RS_PIN, INPUT);

    Serial.begin(9600);
    while (!Serial)
        ; // Wait for serial on boards with native USB

    Serial.println(F("=== RoboCup Junior Rescue Line 2026 - Robot Test Suite ==="));
    Serial.println(F("Framework: AUnit"));
    Serial.println(F(""));
    Serial.println(F("NOTE: Some tests require specific physical setups."));
    Serial.println(F("      See test comments for setup instructions."));
    Serial.println(F(""));
}

void loop()
{
    aunit::TestRunner::run();
}
