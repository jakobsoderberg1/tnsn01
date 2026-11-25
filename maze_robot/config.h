#pragma once

// ======================= WHAT TO RUN =======================
// All false = drive the maze route (the real thing).
#define MOTOR_TEST            false  // left wheel forward, then right wheel forward
#define SENSOR_TEST           false  // motors off, prints sensor readings
#define TEST_LINE_FOLLOW_ONLY true   // test corners first; set false for the maze route

#define DEBUG_PRINT           false  // no cable attached, so no printing


// ======================= PINS (Arduino Leonardo) =======================
#define LEFT_MOTOR_PIN1   11
#define LEFT_MOTOR_PIN2   10
#define RIGHT_MOTOR_PIN1   5
#define RIGHT_MOTOR_PIN2   6   // moved from 9, which is now a sensor

// A wheel turns the wrong way? Set it to true (test first with the
// "reverse" checkboxes on the dashboard, then copy the result here).
#define LEFT_MOTOR_REVERSED   true
#define RIGHT_MOTOR_REVERSED  true

// Line sensors (QTR-8A), index 0 = leftmost seen from behind. A8 = D8, A9 = D9.
// (D3 cannot read analog on the Leonardo, D4/D6/D8/D9/D10/D12 can.)
const int SENSOR_PINS[8] = {A8, A9, A5, A4, A3, A2, A1, A0};

#define LED_PIN 13


// ======================= LINE SENSORS =======================
// A sensor sees black when BLACK_DIRECTION * (reading - THRESHOLD) > 15
// Measured: wood ~120-390, line ~850. Can be tuned live on the dashboard and saved there.
const int THRESHOLD[8]       = {500, 500, 500, 500, 500, 500, 500, 600};
const int BLACK_DIRECTION[8] = {  1,   1,   1,   1,   1,   1,   1,   1};
#define SENSOR_DOUBLE_READ false  // read each sensor twice, keep the 2nd. Measured: no help here; toggle on the dashboard


// ======================= SPEED & STEERING =======================
#define DRIVE_SPEED     90    // a bit below your 100 while testing the new power setup
#define TURN_SPEED     150    // fast part of a turn
#define SEARCH_SPEED    90    // end of a turn, while looking for the next line

#define KP           0.02     // your tuned values
#define KD           0.00

#define LOOP_MS              3
#define SIDE_LINE_READINGS   3
#define LINE_LOST_MS       100   // line must be gone this long (ms) before the robot reacts; shorter = glitch. Adjustable on the dashboard

#define MAX_STEER  40


// ======================= NODES & TURNS =======================
#define CORNER_FORWARD_MS     30   // wheels should stop on the corner
#define MIN_SPIN_MS          150   // fast part of a turn
#define IGNORE_AFTER_TURN_MS  250  // a little extra margin after each node


// ======================= TASK =======================
#define ITEMS_TO_RESCUE 3

// ======================= DEBUG DASHBOARD =======================
#define START_PAUSED       false  // true = wait for "Run" from the dashboard
#define TELEMETRY_MS       50     // dashboard update rate (20 per second)
#define MANUAL_TIMEOUT_MS  300    // manual drive stops if the dashboard goes quiet
