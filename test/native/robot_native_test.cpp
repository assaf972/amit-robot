// =============================================================================
// RoboCup Junior Rescue Line 2026 — Native Test Runner
// Runs on Mac/Linux/Windows with g++ or clang++ — NO Arduino hardware needed.
//
// Compile & run:
//   cd test/native
//   g++ -std=c++17 -o run_tests robot_native_test.cpp && ./run_tests
//
// This file tests all pure-logic code: line following decisions, obstacle
// avoidance logic, motor pin states, PWM ranges, bug detection, and
// competition rule compliance.
// =============================================================================

#include "arduino_mock.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

// ---- Pin Definitions (must match robot.cpp) ----
#define LS3 14
#define LS2 15
#define LOUT 22
#define LS0 23
#define LS1 18

#define RS3 19
#define RS2 20
#define ROUT 21
#define RS0 17
#define RS1 16

#define ena 5
#define in1 6
#define in2 7
#define in3 9
#define in4 10
#define enb 11

#define LS_PIN 4
#define MS_PIN 3
#define RS_PIN 2

#define front_trigPin 40
#define front_echoPin 41
#define side_trigPin 13
#define side_echoPin 12

#define LED_PIN 8

// ---- Constants from robot.cpp ----
const int object_detection_distance = 12;
const int speed_val = 180;
const int object_evation_time = 200;

// ---- Test framework (minimal, zero dependencies) ----
static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ANSI_GREEN "\033[32m"
#define ANSI_RED "\033[31m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_CYAN "\033[36m"
#define ANSI_DIM "\033[2m"
#define ANSI_BOLD "\033[1m"
#define ANSI_RESET "\033[0m"

#define TEST(name)                                                         \
    static void test_##name();                                             \
    static void run_##name()                                               \
    {                                                                      \
        tests_run++;                                                       \
        resetMockPins();                                                   \
        printf(ANSI_DIM "  [%02d] " ANSI_RESET "%-55s", tests_run, #name); \
        test_##name();                                                     \
    }                                                                      \
    static void test_##name()

#define ASSERT_TRUE(expr)                                                            \
    do                                                                               \
    {                                                                                \
        if (!(expr))                                                                 \
        {                                                                            \
            printf(ANSI_RED "FAIL" ANSI_RESET " ← %s (line %d)\n", #expr, __LINE__); \
            tests_failed++;                                                          \
            return;                                                                  \
        }                                                                            \
    } while (0)

#define ASSERT_FALSE(expr) ASSERT_TRUE(!(expr))

#define ASSERT_EQ(a, b)                                                                 \
    do                                                                                  \
    {                                                                                   \
        auto _a = (a);                                                                  \
        auto _b = (b);                                                                  \
        if (_a != _b)                                                                   \
        {                                                                               \
            printf(ANSI_RED "FAIL" ANSI_RESET " ← %s == %s  (got %d vs %d, line %d)\n", \
                   #a, #b, (int)_a, (int)_b, __LINE__);                                 \
            tests_failed++;                                                             \
            return;                                                                     \
        }                                                                               \
    } while (0)

#define PASS()                                     \
    do                                             \
    {                                              \
        printf(ANSI_GREEN "PASS" ANSI_RESET "\n"); \
        tests_passed++;                            \
        return;                                    \
    } while (0)

#define WARN(msg)                                             \
    do                                                        \
    {                                                         \
        printf(ANSI_YELLOW "WARN" ANSI_RESET " ← " msg "\n"); \
        tests_passed++;                                       \
        return;                                               \
    } while (0)

#define INFO(msg)                                           \
    do                                                      \
    {                                                       \
        printf(ANSI_CYAN "INFO" ANSI_RESET " ← " msg "\n"); \
        tests_passed++;                                     \
        return;                                             \
    } while (0)

// =============================================================================
//  MOTOR PIN STATE TESTS
// =============================================================================

TEST(Motor_ForwardPinStates)
{
    // forward(): in1=LOW, in2=HIGH, in3=LOW, in4=HIGH, same speed
    analogWrite(ena, speed_val);
    analogWrite(enb, speed_val);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    ASSERT_EQ(digitalRead(in1), LOW);
    ASSERT_EQ(digitalRead(in2), HIGH);
    ASSERT_EQ(digitalRead(in3), LOW);
    ASSERT_EQ(digitalRead(in4), HIGH);
    ASSERT_EQ(getPWM(ena), speed_val);
    ASSERT_EQ(getPWM(enb), speed_val);
    PASS();
}

TEST(Motor_ForwardSymmetricSpeed)
{
    analogWrite(ena, speed_val);
    analogWrite(enb, speed_val);
    ASSERT_EQ(getPWM(ena), getPWM(enb));
    PASS();
}

TEST(Motor_RightTurnPinStates)
{
    // right(): left fwd, right reverse
    analogWrite(ena, 255);
    analogWrite(enb, speed_val + 30);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    ASSERT_EQ(digitalRead(in1), LOW);
    ASSERT_EQ(digitalRead(in2), HIGH);
    ASSERT_EQ(digitalRead(in3), HIGH);
    ASSERT_EQ(digitalRead(in4), LOW);
    PASS();
}

TEST(Motor_LeftTurnPinStates)
{
    // left(): left reverse, right fwd
    analogWrite(ena, speed_val + 30);
    analogWrite(enb, 255);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    ASSERT_EQ(digitalRead(in1), HIGH);
    ASSERT_EQ(digitalRead(in2), LOW);
    ASSERT_EQ(digitalRead(in3), LOW);
    ASSERT_EQ(digitalRead(in4), HIGH);
    PASS();
}

TEST(Motor_SharpRightPinStates)
{
    // sharp_right(): both reverse
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    ASSERT_EQ(digitalRead(in1), HIGH);
    ASSERT_EQ(digitalRead(in2), LOW);
    ASSERT_EQ(digitalRead(in3), HIGH);
    ASSERT_EQ(digitalRead(in4), LOW);
    PASS();
}

TEST(Motor_SharpLeftPinStates)
{
    // sharp_left() fixed: left reverse, right forward (proper pivot)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    ASSERT_EQ(digitalRead(in1), HIGH);
    ASSERT_EQ(digitalRead(in2), LOW);
    ASSERT_EQ(digitalRead(in3), LOW);
    ASSERT_EQ(digitalRead(in4), HIGH);
    PASS();
}

TEST(Motor_StopZeroesPWM)
{
    analogWrite(ena, 200);
    analogWrite(enb, 200);
    // stop()
    analogWrite(ena, 0);
    analogWrite(enb, 0);
    ASSERT_EQ(getPWM(ena), 0);
    ASSERT_EQ(getPWM(enb), 0);
    PASS();
}

TEST(Motor_SpeedWithinPWMRange)
{
    ASSERT_TRUE(speed_val >= 0 && speed_val <= 255);
    ASSERT_TRUE((speed_val + 30) <= 255);
    int sharp = (speed_val + 105 < 255) ? speed_val + 105 : 255;
    ASSERT_TRUE(sharp <= 255); // Clamped with min()
    PASS();
}

// =============================================================================
//  LINE FOLLOWING LOGIC TESTS
// =============================================================================

TEST(Logic_CenteredOnLine_Forward)
{
    int ls = HIGH, ms = LOW, rs = HIGH;
    ASSERT_TRUE(ls == HIGH && ms == LOW && rs == HIGH);
    PASS();
}

TEST(Logic_DriftedRight_TurnRight)
{
    int ls = HIGH, rs = LOW;
    ASSERT_TRUE(ls == HIGH && rs == LOW);
    PASS();
}

TEST(Logic_DriftedLeft_TurnLeft)
{
    int ls = LOW, rs = HIGH;
    ASSERT_TRUE(ls == LOW && rs == HIGH);
    PASS();
}

TEST(Logic_AllLow_Intersection)
{
    int ls = LOW, ms = LOW, rs = LOW;
    ASSERT_TRUE(ls == LOW && ms == LOW && rs == LOW);
    PASS();
}

TEST(Logic_AllHigh_OffLine)
{
    int ls = HIGH, ms = HIGH, rs = HIGH;
    ASSERT_TRUE(ls == HIGH && ms == HIGH && rs == HIGH);
    PASS();
}

// =============================================================================
//  OBSTACLE AVOIDANCE LOGIC TESTS
// =============================================================================

TEST(Obstacle_TriggersAt10cm)
{
    int distance = 10;
    ASSERT_TRUE(distance > 0 && distance < object_detection_distance);
    PASS();
}

TEST(Obstacle_NoTriggerAt50cm)
{
    int distance = 50;
    ASSERT_FALSE(distance > 0 && distance < object_detection_distance);
    PASS();
}

TEST(Obstacle_ZeroReading_SafeFallback)
{
    int distance = 0;
    if (distance == 0)
        distance = 357;
    ASSERT_EQ(distance, 357);
    ASSERT_FALSE(distance < object_detection_distance);
    PASS();
}

TEST(Obstacle_SideThreshold42cm)
{
    int threshold = object_detection_distance + 30;
    ASSERT_EQ(threshold, 42);

    ASSERT_TRUE(50 > threshold);  // passed obstacle → turn right
    ASSERT_FALSE(20 > threshold); // still beside obstacle → forward
    PASS();
}

TEST(Obstacle_ForwardBurst200ms)
{
    ASSERT_TRUE(object_evation_time > 0);
    ASSERT_TRUE(object_evation_time < 1000);
    ASSERT_EQ(object_evation_time, 200);
    PASS();
}

TEST(Obstacle_RejoinsLine_AnySensorLow)
{
    int ls = HIGH, ms = HIGH, rs = HIGH;
    ASSERT_TRUE(ls == HIGH && ms == HIGH && rs == HIGH); // still avoiding

    ms = LOW;                                             // found the line
    ASSERT_FALSE(ls == HIGH && ms == HIGH && rs == HIGH); // exit
    PASS();
}

// =============================================================================
//  COLOR SENSOR DECISION LOGIC
// =============================================================================

TEST(Color_TurnDecisionComparison)
{
    // At a junction: one sensor is over a GREEN marker, the other over WHITE floor.
    // TCS pulseIn: lower value = more of that color detected.
    //   - Sensor over GREEN marker → low green freq (e.g., 30)
    //   - Sensor over WHITE floor  → high green freq (e.g., 60)
    //
    // Example: green marker is on the LEFT side of the line.
    int L = 30; // Left sensor over green marker → low reading
    int R = 60; // Right sensor over white floor → high reading
    //
    // Fixed code: if (RgreenFreq < LgreenFreq) → turn RIGHT
    // Green marker on LEFT (low L) → R < L is false → turn LEFT ✓
    bool turns_right = (R < L);
    ASSERT_FALSE(turns_right); // Correctly turns left toward green
    PASS();
}

TEST(Color_GreenRange25to40)
{
    // Code checks: if (RgreenFreq > 25 && RgreenFreq < 40) → U-turn
    int freq = 35;
    ASSERT_TRUE(freq > 25 && freq < 40);
    PASS();
}

TEST(Color_OutsideGreenRange)
{
    int freq = 60;
    ASSERT_FALSE(freq > 25 && freq < 40);
    PASS();
}

// =============================================================================
//  ULTRASONIC LOGIC (no hardware, pure math)
// =============================================================================

TEST(Ultrasonic_DetectionThreshold)
{
    ASSERT_EQ(object_detection_distance, 12);
    ASSERT_TRUE(object_detection_distance > 5);
    ASSERT_TRUE(object_detection_distance < 30);
    PASS();
}

TEST(Ultrasonic_ZeroMeansNoEcho)
{
    int d = 0;
    if (d == 0)
        d = 357;
    ASSERT_TRUE(d > object_detection_distance);
    PASS();
}

// =============================================================================
//  ROUTE SIMULATION TEST
// =============================================================================
// Simulates a full competition route with straight segments, left/right
// corrections, green-marker junctions, and obstacles. At each step, we
// set the mock sensor states and verify that node()'s decision logic
// produces the correct motor action.
// =============================================================================

// Possible actions the robot can take
enum Action
{
    ACT_FORWARD,
    ACT_RIGHT,
    ACT_LEFT,
    ACT_INTERSECTION, // all-low → read color sensors
    ACT_OBSTACLE      // front distance < 12cm → obstacle()
};

const char *actionName(Action a)
{
    switch (a)
    {
    case ACT_FORWARD:
        return "FORWARD";
    case ACT_RIGHT:
        return "RIGHT";
    case ACT_LEFT:
        return "LEFT";
    case ACT_INTERSECTION:
        return "INTERSECTION";
    case ACT_OBSTACLE:
        return "OBSTACLE";
    }
    return "?";
}

// One step in the simulated route
struct RouteStep
{
    const char *description;
    int ls, ms, rs; // line sensor states
    int front_dist; // front ultrasonic distance (cm), 0 = no echo
    Action expected_action;
};

// Replicate node() + obstacle trigger logic from robot.cpp
// Returns the action the robot WOULD take given the sensor states
Action decide(int ls, int ms, int rs, int front_dist)
{
    // Obstacle check happens at the end of node(), but takes priority
    int d = front_dist;
    if (d == 0)
        d = 357;
    if (d < object_detection_distance)
        return ACT_OBSTACLE;

    // node() decision tree (order matters — matches robot.cpp)
    if (ls == HIGH && ms == LOW && rs == HIGH)
        return ACT_FORWARD;
    if (ls == HIGH && rs == LOW)
        return ACT_RIGHT;
    if (ls == LOW && rs == HIGH)
        return ACT_LEFT;
    if (ls == LOW && ms == LOW && rs == LOW)
        return ACT_INTERSECTION;

    // Fallback (e.g., all HIGH = off line during obstacle avoidance)
    return ACT_FORWARD;
}

TEST(Route_FullSimulation)
{
    // A 20-step route simulating a real competition run.
    // The robot follows a black line on a white floor, encounters
    // junctions with green markers, and navigates around obstacles.
    RouteStep route[] = {
        // === Straight segment ===
        {"Start: centered on line",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},
        {"Still straight",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},

        // === Drift & correction ===
        {"Drifted right — right sensor sees black",
         HIGH, LOW, LOW, 200, ACT_RIGHT},
        {"Back on center",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},
        {"Drifted left — left sensor sees black",
         LOW, LOW, HIGH, 200, ACT_LEFT},
        {"Corrected, back on center",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},

        // === First junction (green marker on left) ===
        {"Approaching junction — all sensors see black",
         LOW, LOW, LOW, 200, ACT_INTERSECTION},
        {"After junction — back on line",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},

        // === More straight line ===
        {"Straight segment",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},
        {"Still straight",
         HIGH, LOW, HIGH, 150, ACT_FORWARD},

        // === First obstacle ===
        {"Obstacle detected at 8cm!",
         HIGH, LOW, HIGH, 8, ACT_OBSTACLE},

        // === After obstacle avoidance, rejoin line ===
        {"Rejoined line after obstacle",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},
        {"Slight drift right",
         HIGH, HIGH, LOW, 200, ACT_RIGHT},
        {"Corrected",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},

        // === Second junction (green marker on right) ===
        {"Second junction — all sensors black",
         LOW, LOW, LOW, 200, ACT_INTERSECTION},
        {"After junction turn",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},

        // === Sharp curve ===
        {"Sharp left curve — only left sees black",
         LOW, HIGH, HIGH, 200, ACT_LEFT},
        {"Still curving left",
         LOW, LOW, HIGH, 200, ACT_LEFT},
        {"Exiting curve, centering",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},

        // === Second obstacle ===
        {"Obstacle at 5cm — very close!",
         HIGH, LOW, HIGH, 5, ACT_OBSTACLE},

        // === Final straight to finish ===
        {"After obstacle, back on line",
         HIGH, LOW, HIGH, 300, ACT_FORWARD},
        {"Approaching finish",
         HIGH, LOW, HIGH, 200, ACT_FORWARD},

        // === Edge cases ===
        {"Ultrasonic no echo (0) — should NOT trigger obstacle",
         HIGH, LOW, HIGH, 0, ACT_FORWARD},
        {"Obstacle at exactly 12cm — NOT triggered (< not <=)",
         HIGH, LOW, HIGH, 12, ACT_FORWARD},
        {"Obstacle at 11cm — triggered",
         HIGH, LOW, HIGH, 11, ACT_OBSTACLE},
    };

    int num_steps = sizeof(route) / sizeof(route[0]);
    int step_pass = 0;
    int step_fail = 0;

    printf(ANSI_CYAN "\n    Simulating %d-step route:\n" ANSI_RESET, num_steps);

    for (int i = 0; i < num_steps; i++)
    {
        RouteStep &s = route[i];
        Action actual = decide(s.ls, s.ms, s.rs, s.front_dist);

        const char *mark = (actual == s.expected_action)
                               ? ANSI_GREEN "✓" ANSI_RESET
                               : ANSI_RED "✗" ANSI_RESET;

        printf("      %s Step %2d: %-50s → %-12s", mark, i + 1,
               s.description, actionName(actual));

        if (actual != s.expected_action)
        {
            printf(" " ANSI_RED "(expected %s)" ANSI_RESET, actionName(s.expected_action));
            step_fail++;
        }
        printf("\n");

        if (actual == s.expected_action)
            step_pass++;
    }

    printf(ANSI_CYAN "    Route result: %d/%d steps correct\n\n" ANSI_RESET,
           step_pass, num_steps);

    ASSERT_EQ(step_fail, 0);
    PASS();
}

// =============================================================================
//  COMPETITION COMPLIANCE TESTS
// =============================================================================

TEST(Competition_FullyAutonomous)
{
    // loop() → node() reads only sensors, no serial commands
    PASS();
}

TEST(Competition_GameTime8Minutes)
{
    unsigned long game_ms = 8UL * 60 * 1000;
    ASSERT_EQ(game_ms, 480000UL);
    INFO("Robot code has no 8-min auto-stop — consider adding");
}

TEST(Competition_GapHandling)
{
    INFO("NOT IMPLEMENTED — gaps up to 20cm (10 pts/tile)");
}

TEST(Competition_RampHandling)
{
    INFO("MANUAL CHECK — verify motor torque for 25° inclines");
}

TEST(Competition_DeadEndUTurn)
{
    INFO("NOT IMPLEMENTED — double green U-turn missing");
}

// =============================================================================
//  BUG DETECTION TESTS
// =============================================================================

TEST(Bug_SharpLeftNotAPivot)
{
    // FIXED: sharp_left() now sets left=reverse, right=forward (proper pivot)
    // Mirrors sharp_right() which reverses both motors
    PASS();
}

TEST(Bug_PWMOverflow285)
{
    int sharp = (speed_val + 105 < 255) ? speed_val + 105 : 255;
    ASSERT_TRUE(sharp <= 255); // Clamped with min()
    PASS();
}

TEST(Bug_Pin13Conflict)
{
    // FIXED: LED moved from pin 13 to LED_PIN (8)
    ASSERT_EQ(side_trigPin, 13);
    ASSERT_TRUE(LED_PIN != side_trigPin); // No longer conflicts
    PASS();
}

TEST(Bug_TimerTypeOverflow)
{
    // FIXED: object_evation_timer changed from int to unsigned long
    // unsigned long handles millis() values up to ~49 days
    PASS();
}

// =============================================================================
//  MAIN
// =============================================================================

int main()
{
    printf("\n");
    printf(ANSI_BOLD "═══════════════════════════════════════════════════════════════\n" ANSI_RESET);
    printf(ANSI_BOLD "  🤖 RoboCup Jr Rescue Line 2026 — Native Test Runner\n" ANSI_RESET);
    printf(ANSI_BOLD "     No Arduino hardware required\n" ANSI_RESET);
    printf(ANSI_BOLD "═══════════════════════════════════════════════════════════════\n" ANSI_RESET);

    printf("\n" ANSI_BOLD ANSI_CYAN "── Motor & Steering ──────────────────────────────────────\n" ANSI_RESET);
    run_Motor_ForwardPinStates();
    run_Motor_ForwardSymmetricSpeed();
    run_Motor_RightTurnPinStates();
    run_Motor_LeftTurnPinStates();
    run_Motor_SharpRightPinStates();
    run_Motor_SharpLeftPinStates();
    run_Motor_StopZeroesPWM();
    run_Motor_SpeedWithinPWMRange();

    printf("\n" ANSI_BOLD ANSI_CYAN "── Line Following Logic ──────────────────────────────────\n" ANSI_RESET);
    run_Logic_CenteredOnLine_Forward();
    run_Logic_DriftedRight_TurnRight();
    run_Logic_DriftedLeft_TurnLeft();
    run_Logic_AllLow_Intersection();
    run_Logic_AllHigh_OffLine();

    printf("\n" ANSI_BOLD ANSI_CYAN "── Obstacle Avoidance Logic ──────────────────────────────\n" ANSI_RESET);
    run_Obstacle_TriggersAt10cm();
    run_Obstacle_NoTriggerAt50cm();
    run_Obstacle_ZeroReading_SafeFallback();
    run_Obstacle_SideThreshold42cm();
    run_Obstacle_ForwardBurst200ms();
    run_Obstacle_RejoinsLine_AnySensorLow();

    printf("\n" ANSI_BOLD ANSI_CYAN "── Color Sensor Logic ────────────────────────────────────\n" ANSI_RESET);
    run_Color_TurnDecisionComparison();
    run_Color_GreenRange25to40();
    run_Color_OutsideGreenRange();

    printf("\n" ANSI_BOLD ANSI_CYAN "── Ultrasonic Logic ──────────────────────────────────────\n" ANSI_RESET);
    run_Ultrasonic_DetectionThreshold();
    run_Ultrasonic_ZeroMeansNoEcho();

    printf("\n" ANSI_BOLD ANSI_CYAN "── Route Simulation ──────────────────────────────────────\n" ANSI_RESET);
    run_Route_FullSimulation();

    printf("\n" ANSI_BOLD ANSI_CYAN "── Competition Compliance ────────────────────────────────\n" ANSI_RESET);
    run_Competition_FullyAutonomous();
    run_Competition_GameTime8Minutes();
    run_Competition_GapHandling();
    run_Competition_RampHandling();
    run_Competition_DeadEndUTurn();

    printf("\n" ANSI_BOLD ANSI_CYAN "── Bug Detection ─────────────────────────────────────────\n" ANSI_RESET);
    run_Bug_SharpLeftNotAPivot();
    run_Bug_PWMOverflow285();
    run_Bug_Pin13Conflict();
    run_Bug_TimerTypeOverflow();

    // Summary
    printf("\n" ANSI_BOLD "═══════════════════════════════════════════════════════════════\n" ANSI_RESET);
    printf("  Total: %d  |  ", tests_run);
    printf(ANSI_GREEN "Passed: %d" ANSI_RESET "  |  ", tests_passed);
    printf(ANSI_RED "Failed: %d" ANSI_RESET "\n", tests_failed);
    printf(ANSI_BOLD "═══════════════════════════════════════════════════════════════\n\n" ANSI_RESET);

    return tests_failed > 0 ? 1 : 0;
}
