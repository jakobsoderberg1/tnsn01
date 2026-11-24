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
#define RIGHT_MOTOR_PIN2   9

// Line sensors (QTR-8A), index 0 = leftmost seen from behind. A6 = D4, A7 = D6.
const int SENSOR_PINS[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

#define LED_PIN 13


// ======================= LINE SENSORS =======================
// A sensor sees black when BLACK_DIRECTION * (reading - THRESHOLD) > 15
// Measured: wood ~120-390, line ~850.
const int THRESHOLD[8]       = {500, 500, 500, 500, 500, 500, 500, 600};
const int BLACK_DIRECTION[8] = {  1,   1,   1,   1,   1,   1,   1,   1};


// ======================= SPEED & STEERING =======================
#define DRIVE_SPEED     90    // a bit below your 100 while testing the new power setup
#define TURN_SPEED     150    // fast part of a turn
#define SEARCH_SPEED    90    // end of a turn, while looking for the next line

#define KP           0.02     // your tuned values
#define KD           0.00

#define LOOP_MS              3
#define SIDE_LINE_READINGS   3

#define MAX_STEER  40


// ======================= NODES & TURNS =======================
#define CORNER_FORWARD_MS     30   // wheels should stop on the corner
#define MIN_SPIN_MS          150   // fast part of a turn
#define IGNORE_AFTER_TURN_MS  250  // a little extra margin after each node


// ======================= TASK =======================
#define ITEMS_TO_RESCUE 3