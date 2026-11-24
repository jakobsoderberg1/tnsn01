/*
  SEARCH & RESCUE MAZE ROBOT

  The maze map is known, so the robot does not explore. It follows a
  pre-planned route (route.h, made by plan_route.py) that drives over every
  line in the maze, using only the line sensors:

    - side line under the sensors  -> we reached the next node (corner/junction)
    - line ends                    -> we reached a dead end
    - the map says what to do there: straight, left, right or turn around

  After ITEMS_TO_RESCUE items it takes the shortest way back to the start.
*/

#include "config.h"
#include "route.h"

// Defaults for settings that older config.h files may not have
#ifndef LOOP_MS
#define LOOP_MS 3               // control loop period in ms
#endif
#ifndef SIDE_LINE_READINGS
#define SIDE_LINE_READINGS 3    // readings in a row before a side line counts
#endif
#ifndef DEBUG_PRINT
#define DEBUG_PRINT false
#endif
#ifndef MAX_STEER
#define MAX_STEER 60            // largest speed difference between the wheels when steering
#endif


// =====================================================================
// MOTORS
// =====================================================================

// Your motors run "backwards": forward = pin1 LOW, PWM on pin2
void setMotor(int pin1, int pin2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pin1, LOW);
    analogWrite(pin2, speed);
  } else if (speed < 0) {
    analogWrite(pin1, -speed);
    digitalWrite(pin2, LOW);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, leftSpeed);
  setMotor(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, rightSpeed);
}

void stopMotors() {
  drive(0, 0);
}


// =====================================================================
// LINE SENSORS
// =====================================================================

bool black[8];          // does sensor i see the line?
int  strength[8];       // how strongly (0 = not at all)
int  lastLineError = 0;

void readSensors() {
  for (int i = 0; i < 8; i++) {
    int signal = BLACK_DIRECTION[i] * (analogRead(SENSOR_PINS[i]) - THRESHOLD[i]);
    black[i] = signal > 15;
    strength[i] = black[i] ? signal : 0;
  }
}

// Where is the line? -2500 = far left, 0 = centred, +2500 = far right.
// Uses the six inner sensors.
int lineError() {
  long weightedSum = 0;
  long total = 0;
  for (int i = 1; i <= 6; i++) {
    weightedSum += (long)strength[i] * (i - 1) * 1000;
    total += strength[i];
  }
  if (total > 0) {
    lastLineError = weightedSum / total - 2500;
  }
  return lastLineError;    // keep the last known value if the line is gone
}

bool lineVisible() { for (int i = 1; i <= 6; i++) if (black[i]) return true; return false; }
bool lineCentred() { return black[3] || black[4]; }

// How many of sensors from..to see black
int blackCount(int from, int to) {
  int count = 0;
  for (int i = from; i <= to; i++) if (black[i]) count++;
  return count;
}

// A straight line covers at most 2-3 sensors. A side line (corner, T, cross)
// covers 4 or more, so that is how we spot a node.
bool sideLineSeen() { return blackCount(0, 7) >= 4; }
bool sideLineIsLeft() { return blackCount(0, 3) >= blackCount(4, 7); }


// =====================================================================
// LINE FOLLOWING & TURNING
// =====================================================================

int lastError = 0;

// Print the sensor pattern, e.g. "00011000"
void printPattern() {
  for (int i = 0; i < 8; i++) Serial.print(black[i] ? "1" : "0");
}

// One step of PD line following
void followLine() {
  int error = lineError();

  // The error jumps in steps when the line moves to the next sensor. Smooth
  // the change a little so KD damps the swinging instead of giving kicks.
  static float smoothChange = 0;
  smoothChange = 0.7 * smoothChange + 0.3 * (error - lastError);
  lastError = error;

  int correction = KP * error + KD * smoothChange;
  correction = constrain(correction, -MAX_STEER, MAX_STEER);   // never steer too hard
  int leftSpeed  = DRIVE_SPEED + correction;
  int rightSpeed = DRIVE_SPEED - correction;
  drive(leftSpeed, rightSpeed);

  // Every 100 ms: what the sensors see and what the motors get
  static unsigned long lastPrint = 0;
  if (DEBUG_PRINT && millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("sensors ");   printPattern();
    Serial.print("  error ");   Serial.print(error);
    Serial.print("  steer ");   Serial.print(correction);
    Serial.print("  L ");       Serial.print(leftSpeed);
    Serial.print("  R ");       Serial.println(rightSpeed);
  }
}

// Turn on the spot until the next line is under the middle sensors
void spinUntilLine(bool turnLeft) {
  int side = turnLeft ? 1 : -1;

  // Fast part: get off the line we are standing on. Spin at least
  // MIN_SPIN_MS, and keep going until the middle sensors are off the line.
  drive(-side * TURN_SPEED, side * TURN_SPEED);
  delay(MIN_SPIN_MS);
  unsigned long leaveStart = millis();
  do { readSensors(); } while (lineCentred() && millis() - leaveStart < 1000);

  // Slower part: look for the next line so we don't spin past it
  drive(-side * SEARCH_SPEED, side * SEARCH_SPEED);
  unsigned long start = millis();
  bool found = false;
  while (millis() - start < 3000) {
    readSensors();
    if (lineCentred()) { found = true; break; }
  }
  stopMotors();
  if (DEBUG_PRINT) Serial.println(found ? "   turn done, line found" : "   turn FAILED, no line found in 3 s");
  lastError = 0;
  lastLineError = 0;
}


// =====================================================================
// MAP
// =====================================================================

const uint8_t NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3;

Node getNode(uint8_t index) {
  Node node;
  memcpy_P(&node, &NODES[index], sizeof(Node));
  return node;
}

// Which direction (N/E/S/W) do we leave 'from' to reach 'to'?
uint8_t directionBetween(uint8_t from, uint8_t to) {
  Node node = getNode(from);
  for (uint8_t dir = 0; dir < 4; dir++) {
    if (node.nb[dir] == to) return dir;
  }
  return NO_NODE;
}

bool hasLine(uint8_t node, uint8_t dir) { return getNode(node).nb[dir] != NO_NODE; }
bool isDeadEnd(uint8_t node)            { return getNode(node).flags & NF_DEADEND; }

// Shortest way home from every node (Dijkstra, run once at startup).
// stepTowardBase[n] = the next node to drive to from n.
uint16_t distanceToBase[NUM_NODES];
uint8_t  stepTowardBase[NUM_NODES];

void computeWayHome() {
  bool finished[NUM_NODES];
  for (int i = 0; i < NUM_NODES; i++) {
    distanceToBase[i] = 0xFFFF;
    stepTowardBase[i] = NO_NODE;
    finished[i] = false;
  }
  distanceToBase[BASE_NODE] = 0;

  for (int step = 0; step < NUM_NODES; step++) {
    // pick the closest unfinished node
    int current = -1;
    for (int i = 0; i < NUM_NODES; i++) {
      if (!finished[i] && (current < 0 || distanceToBase[i] < distanceToBase[current])) current = i;
    }
    finished[current] = true;

    // update its neighbours
    Node node = getNode(current);
    for (int dir = 0; dir < 4; dir++) {
      uint8_t neighbour = node.nb[dir];
      if (neighbour == NO_NODE) continue;
      uint16_t viaCurrent = distanceToBase[current] + node.len[dir];
      if (viaCurrent < distanceToBase[neighbour]) {
        distanceToBase[neighbour] = viaCurrent;
        stepTowardBase[neighbour] = current;
      }
    }
  }
}


// =====================================================================
// RESCUE (TODO: sensor + gripper)
// =====================================================================

bool itemAhead() {
  return false;     // TODO: e.g. IR distance sensor sees a cylinder close in front
}

void pickUpItem() {
  delay(500);       // TODO: gripper. Leave the robot on the line, same heading.
}


// =====================================================================
// MAZE RUN
// =====================================================================

// The path we follow: a list of nodes. Starts as the full route, gets
// replaced by the way home after the last item.
uint8_t path[ROUTE_LEN];
int pathLength = 0;
int pathIndex = 0;          // we are driving from path[pathIndex] to path[pathIndex + 1]
bool goingHome = false;
bool routeDone = false;

int itemsCarried = 0;
int lostCount = 0, sideLineCount = 0;   // to ignore single noisy readings
unsigned long ignoreSideLinesUntil = 0; // right after a node, don't count it twice

void loadRoute() {
  for (int i = 0; i < ROUTE_LEN; i++) path[i] = pgm_read_byte(&ROUTE[i]);
  pathLength = ROUTE_LEN;
  pathIndex = 0;
}

// Replace the path with the shortest way from 'node' to the start
void headHome(uint8_t node) {
  pathLength = 0;
  for (uint8_t n = node; n != NO_NODE; n = stepTowardBase[n]) {
    path[pathLength++] = n;
  }
  pathIndex = 0;
  goingHome = true;
  Serial.print("  [all items found, going home]");
}

// We just reached the next node. 'deadEnd' = the line ended (no side line).
void nodeReached(bool deadEnd) {
  uint8_t node = path[pathIndex + 1];
  uint8_t arriving = directionBetween(path[pathIndex], node);
  Serial.print("At "); Serial.print(NODE_NAMES[node]);

  // Move one step along the path, or switch to the way home
  if (itemsCarried >= ITEMS_TO_RESCUE && !goingHome) headHome(node);
  else pathIndex++;

  // Now path[pathIndex] is this node
  if (pathIndex >= pathLength - 1) {
    Serial.println("  -> finished, back at the start!");
    stopMotors();
    routeDone = true;
    return;
  }

  uint8_t leaving = directionBetween(node, path[pathIndex + 1]);
  int turnType = (leaving - arriving + 4) % 4;   // 0 straight, 1 right, 2 back, 3 left
  const char* names[] = {"straight", "RIGHT", "turn around", "LEFT"};
  Serial.print("  -> "); Serial.println(names[turnType]);

  if (turnType != 0) {
    if (!deadEnd) {                     // drive the wheels onto the node first
      drive(DRIVE_SPEED, DRIVE_SPEED);
      delay(CORNER_FORWARD_MS);
    }
    stopMotors();

    if (turnType == 1) spinUntilLine(false);   // right
    if (turnType == 3) spinUntilLine(true);    // left
    if (turnType == 2) {                       // turn around (left way round)
      uint8_t leftSide = (arriving + 3) % 4;
      if (hasLine(node, leftSide)) spinUntilLine(true);   // pass the left branch first
      spinUntilLine(true);
    }
  }

  sideLineCount = 0;
  lostCount = 0;
  ignoreSideLinesUntil = millis() + IGNORE_AFTER_TURN_MS;
}

// Runs every LOOP_MS
void mazeRun() {
  if (routeDone) { stopMotors(); digitalWrite(LED_PIN, HIGH); return; }

  readSensors();

  if (itemAhead()) {
    stopMotors();
    pickUpItem();
    itemsCarried++;
    Serial.print("Item "); Serial.println(itemsCarried);
  }

  bool deadEndAhead = isDeadEnd(path[pathIndex + 1]);
  sideLineCount = sideLineSeen() ? sideLineCount + 1 : 0;
  lostCount     = lineVisible()  ? 0 : lostCount + 1;

  // Side line in 3 readings in a row = corner or junction
  // (a dead end has no side lines, so ignore them there)
  if (sideLineCount >= SIDE_LINE_READINGS && millis() > ignoreSideLinesUntil && !deadEndAhead) {
    Serial.print("SIDE LINE ");  printPattern();  Serial.print("  -> ");
    nodeReached(false);
    return;
  }

  // Line ended = dead end (or a corner whose side line we missed)
  if (lostCount >= 3) {
    Serial.print("LINE LOST ");  printPattern();  Serial.print("  -> ");
    nodeReached(deadEndAhead);
    return;
  }

  followLine();
}


// =====================================================================
// TEST MODES
// =====================================================================

// SENSOR_TEST: motors off, prints raw readings, black pattern and line error
void printSensors() {
  readSensors();
  Serial.print("raw:");
  for (int i = 0; i < 8; i++) { Serial.print(" "); Serial.print(analogRead(SENSOR_PINS[i])); }
  Serial.print("   black: ");
  for (int i = 0; i < 8; i++) Serial.print(black[i] ? "1" : "0");
  Serial.print("   error: ");
  Serial.println(lineError());
  delay(200);
}

// MOTOR_TEST: left wheel forward, then right wheel forward
void motorTest() {
  Serial.println("LEFT wheel should turn FORWARD");
  drive(150, 0);  delay(1500);  stopMotors();  delay(1000);
  Serial.println("RIGHT wheel should turn FORWARD");
  drive(0, 150);  delay(1500);  stopMotors();  delay(2000);
}

// TEST_LINE_FOLLOW_ONLY: follow the line, turn LEFT at every corner/junction
void testLineFollow() {
  readSensors();
  sideLineCount = sideLineSeen() ? sideLineCount + 1 : 0;

  if (sideLineCount >= SIDE_LINE_READINGS && millis() > ignoreSideLinesUntil) {
    bool left = sideLineIsLeft();
    Serial.print(left ? "corner LEFT  " : "corner RIGHT ");  printPattern();  Serial.println();
    drive(DRIVE_SPEED, DRIVE_SPEED);
    delay(CORNER_FORWARD_MS);
    stopMotors();
    spinUntilLine(left);
    sideLineCount = 0;
    ignoreSideLinesUntil = millis() + IGNORE_AFTER_TURN_MS;
    return;
  }

  if (!lineVisible()) {                       // overshot: turn back to the line
    Serial.print("line lost    ");  printPattern();  Serial.println();
    spinUntilLine(lastLineError < 0);
    ignoreSideLinesUntil = millis() + IGNORE_AFTER_TURN_MS;
    return;
  }

  followLine();
}


// =====================================================================
// SETUP & LOOP
// =====================================================================

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT); pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  stopMotors();

  // Blink 3 times on every start. If you see this in the middle of a run,
  // the Arduino reset itself (usually the battery voltage dropped).
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(150);
    digitalWrite(LED_PIN, LOW);  delay(150);
  }
  Serial.println("Program started");

  computeWayHome();
  loadRoute();

  delay(3000);   // 3 s to put the robot down: at the start, on the line, facing WEST
}

void loop() {
  static unsigned long lastRun = 0;
  if (millis() - lastRun < LOOP_MS) return;
  lastRun = millis();

  if (MOTOR_TEST)                 motorTest();
  else if (SENSOR_TEST)           printSensors();
  else if (TEST_LINE_FOLLOW_ONLY) testLineFollow();
  else                            mazeRun();
}