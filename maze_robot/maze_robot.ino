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
#include <EEPROM.h>

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
#ifndef LEFT_MOTOR_REVERSED
#define LEFT_MOTOR_REVERSED false
#endif
#ifndef RIGHT_MOTOR_REVERSED
#define RIGHT_MOTOR_REVERSED false
#endif
#ifndef LINE_LOST_MS
#define LINE_LOST_MS 40
#endif
#ifndef SENSOR_DOUBLE_READ
#define SENSOR_DOUBLE_READ true
#endif
#ifndef START_PAUSED
#define START_PAUSED false
#endif
#ifndef TELEMETRY_MS
#define TELEMETRY_MS 50
#endif
#ifndef MANUAL_TIMEOUT_MS
#define MANUAL_TIMEOUT_MS 300
#endif


// =====================================================================
// RUN STATE (controlled from the debug dashboard, see dashboard/)
// =====================================================================

#define MODE_MAZE        0
#define MODE_LINE_TEST   1
#define MODE_SENSORS     2
#define MODE_MANUAL      3
#define MODE_MOTOR_TEST  4

uint8_t mode = MOTOR_TEST            ? MODE_MOTOR_TEST
             : SENSOR_TEST           ? MODE_SENSORS
             : TEST_LINE_FOLLOW_ONLY ? MODE_LINE_TEST
             :                         MODE_MAZE;

bool paused = START_PAUSED;     // freeze the program (motors off) wherever it is
bool abortRequested = false;    // mode change / reset: leave any running turn
bool resetPending = false;
bool stepping = false;          // running one step, then pause again
unsigned long stepEnd = 0;      // in robot time
bool pauseAtNextNode = false;   // "run to next node"
bool breakAtNodes = false;      // pause at every node
int  speedScale = 100;          // slow motion, % of normal speed

bool telemetryOn = false;       // turned on when the dashboard says hello
int  manualLeft = 0, manualRight = 0;
unsigned long manualUntil = 0;

// Modes where pause / step / slow motion apply
bool pausable() { return mode == MODE_MAZE || mode == MODE_LINE_TEST || mode == MODE_MOTOR_TEST; }

// "Robot time": stops while paused and runs slower in slow motion, so
// time-based moves (spin MIN_SPIN_MS, CORNER_FORWARD_MS...) still cover
// the same distance at lower speed.
unsigned long robotTimeX100 = 0;
unsigned long lastTimeUpdate = 0;
unsigned long runMillis() {
  unsigned long now = millis();
  unsigned long dt = now - lastTimeUpdate;
  lastTimeUpdate = now;
  if (!(paused && pausable())) robotTimeX100 += dt * (pausable() ? speedScale : 100);
  return robotTimeX100 / 100;
}

void debugService();   // defined in the DEBUG LINK section
void nodeEvent();

// delay() that keeps the dashboard alive and respects pause / slow motion
void waitMs(unsigned long ms) {
  unsigned long start = runMillis();
  while (runMillis() - start < ms && !abortRequested) debugService();
}


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

bool leftReversed  = LEFT_MOTOR_REVERSED;    // flip a wheel's direction (config.h / dashboard)
bool rightReversed = RIGHT_MOTOR_REVERSED;
int requestedLeft = 0, requestedRight = 0;   // what the program asks for
int outputLeft = 0, outputRight = 0;         // what the motors actually get

void applyMotors() {
  bool hold = paused && pausable();
  int scale = pausable() ? speedScale : 100;
  outputLeft  = hold ? 0 : (long)requestedLeft  * scale / 100;
  outputRight = hold ? 0 : (long)requestedRight * scale / 100;
  // outputLeft/Right stay "positive = forward"; only the pins get flipped
  setMotor(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, leftReversed ? -outputLeft : outputLeft);
  setMotor(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, rightReversed ? -outputRight : outputRight);
}

void drive(int leftSpeed, int rightSpeed) {
  requestedLeft  = constrain(leftSpeed, -255, 255);
  requestedRight = constrain(rightSpeed, -255, 255);
  applyMotors();
}

void stopMotors() {
  drive(0, 0);
}


// =====================================================================
// LINE SENSORS
// =====================================================================

bool black[8];          // does sensor i see the line?
int  strength[8];       // how strongly (0 = not at all)
int  raw[8];            // last analogRead values (for the dashboard)

// Thresholds can be changed from the dashboard and saved in EEPROM.
// Saved values are only used while THRESHOLD in config.h is unchanged,
// so editing config.h always wins over old saved values.
int threshold[8];
extern int lineLostMs;
const uint16_t EEPROM_MAGIC = 0x7E52;   // change when SavedThresholds changes

struct SavedThresholds {
  uint16_t magic;
  int      configDefaults[8];   // THRESHOLD at the time of saving
  int      values[8];
  int      lineLostMs;
};

bool loadThresholds() {
  for (int i = 0; i < 8; i++) threshold[i] = THRESHOLD[i];
  SavedThresholds saved;
  EEPROM.get(0, saved);
  if (saved.magic != EEPROM_MAGIC) return false;
  for (int i = 0; i < 8; i++) if (saved.configDefaults[i] != THRESHOLD[i]) return false;
  for (int i = 0; i < 8; i++) threshold[i] = saved.values[i];
  if (saved.lineLostMs >= 0 && saved.lineLostMs <= 2000) lineLostMs = saved.lineLostMs;
  return true;
}

void saveThresholds() {
  SavedThresholds saved;
  saved.magic = EEPROM_MAGIC;
  for (int i = 0; i < 8; i++) { saved.configDefaults[i] = THRESHOLD[i]; saved.values[i] = threshold[i]; }
  saved.lineLostMs = lineLostMs;
  EEPROM.put(0, saved);   // only writes bytes that changed
}
int  lastLineError = 0;

// The QTR-8A outputs are too weak for the ADC to switch between them at full
// speed, so a reading partly copies the previous sensor. Reading twice and
// keeping the second gives the input time to settle.
bool doubleRead = SENSOR_DOUBLE_READ;

void readSensors() {
  for (int i = 0; i < 8; i++) {
    if (doubleRead) analogRead(SENSOR_PINS[i]);
    raw[i] = analogRead(SENSOR_PINS[i]);
    int signal = BLACK_DIRECTION[i] * (raw[i] - threshold[i]);
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

// The line only counts as lost after it has been gone for LINE_LOST_MS.
// Shorter dropouts (glitches) are ignored but counted, so the dashboard
// can show how often they happen.
bool lineGone = false;
unsigned long lineGoneSince = 0;
unsigned int lineBlips = 0;
int lineLostMs = LINE_LOST_MS;          // adjustable from the dashboard

bool lineLost() {
  if (lineVisible()) {
    if (lineGone && runMillis() - lineGoneSince < (unsigned long)lineLostMs) lineBlips++;
    lineGone = false;
    return false;
  }
  if (!lineGone) { lineGone = true; lineGoneSince = runMillis(); }
  return runMillis() - lineGoneSince >= (unsigned long)lineLostMs;
}
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
int lastCorrection = 0;

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
  lastCorrection = correction;
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
  waitMs(MIN_SPIN_MS);
  unsigned long leaveStart = runMillis();
  do { debugService(); readSensors(); }
  while (lineCentred() && runMillis() - leaveStart < 1000 && !abortRequested);

  // Slower part: look for the next line so we don't spin past it
  drive(-side * SEARCH_SPEED, side * SEARCH_SPEED);
  unsigned long start = runMillis();
  bool found = false;
  while (runMillis() - start < 3000 && !abortRequested) {
    debugService();
    readSensors();
    if (lineCentred()) { found = true; break; }
  }
  stopMotors();
  if (DEBUG_PRINT) Serial.println(found ? "   turn done, line found" : "   turn FAILED, no line found in 3 s");
  lastError = 0;
  lastLineError = 0;
  lastCorrection = 0;
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
  waitMs(500);      // TODO: gripper. Leave the robot on the line, same heading.
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
int sideLineCount = 0;                  // to ignore single noisy readings
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
  nodeEvent();                          // dashboard: maybe pause here, before the turn

  if (turnType != 0) {
    if (!deadEnd) {                     // drive the wheels onto the node first
      drive(DRIVE_SPEED, DRIVE_SPEED);
      waitMs(CORNER_FORWARD_MS);
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
  lineGone = false;
  ignoreSideLinesUntil = runMillis() + IGNORE_AFTER_TURN_MS;
}

void resetMaze() {
  loadRoute();
  goingHome = false;
  routeDone = false;
  itemsCarried = 0;
  sideLineCount = 0;
  lineGone = false;
  ignoreSideLinesUntil = 0;
  lastError = lastLineError = lastCorrection = 0;
  digitalWrite(LED_PIN, LOW);
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
  bool lost = lineLost();

  // Side line in 3 readings in a row = corner or junction
  // (a dead end has no side lines, so ignore them there)
  if (sideLineCount >= SIDE_LINE_READINGS && runMillis() > ignoreSideLinesUntil && !deadEndAhead) {
    Serial.print("SIDE LINE ");  printPattern();  Serial.print("  -> ");
    nodeReached(false);
    return;
  }

  // Line ended = dead end (or a corner whose side line we missed)
  if (lost) {
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
  stopMotors();
  lineError();
  static unsigned long lastPrint = 0;
  if (telemetryOn || millis() - lastPrint < 200) return;   // dashboard shows it instead
  lastPrint = millis();
  Serial.print("raw:");
  for (int i = 0; i < 8; i++) { Serial.print(" "); Serial.print(raw[i]); }
  Serial.print("   black: ");
  for (int i = 0; i < 8; i++) Serial.print(black[i] ? "1" : "0");
  Serial.print("   error: ");
  Serial.println(lastLineError);
}

// MOTOR_TEST: left wheel forward, then right wheel forward
void motorTest() {
  Serial.println("LEFT wheel should turn FORWARD");
  drive(150, 0);  waitMs(1500);  stopMotors();  waitMs(1000);
  if (abortRequested) return;
  Serial.println("RIGHT wheel should turn FORWARD");
  drive(0, 150);  waitMs(1500);  stopMotors();  waitMs(2000);
}

// TEST_LINE_FOLLOW_ONLY: follow the line, turn LEFT at every corner/junction
void testLineFollow() {
  readSensors();
  sideLineCount = sideLineSeen() ? sideLineCount + 1 : 0;

  if (sideLineCount >= SIDE_LINE_READINGS && runMillis() > ignoreSideLinesUntil) {
    bool left = sideLineIsLeft();
    Serial.print(left ? "corner LEFT  " : "corner RIGHT ");  printPattern();  Serial.println();
    nodeEvent();
    drive(DRIVE_SPEED, DRIVE_SPEED);
    waitMs(CORNER_FORWARD_MS);
    stopMotors();
    spinUntilLine(left);
    sideLineCount = 0;
    lineGone = false;
    ignoreSideLinesUntil = runMillis() + IGNORE_AFTER_TURN_MS;
    return;
  }

  if (lineLost()) {                           // overshot: turn back to the line
    Serial.print("line lost    ");  printPattern();  Serial.println();
    nodeEvent();
    spinUntilLine(lastLineError < 0);
    lineGone = false;
    ignoreSideLinesUntil = runMillis() + IGNORE_AFTER_TURN_MS;
    return;
  }

  followLine();
}


// =====================================================================
// DEBUG LINK (USB serial <-> dashboard/index.html)
//
// Robot -> PC, one line each, space separated:
//   T ms mode flags raw0..raw7 blackbits error steer reqL reqR outL outR
//     scale fromNode toNode items pathIndex pathLength lineBlips
//   C thr0..thr7 dir0..dir7 DRIVE_SPEED TURN_SPEED SEARCH_SPEED default0..default7 lineLostMs
//   anything else = log text
//
// PC -> robot, one command per line:
//   ?        hello: start telemetry, send config      q  stop telemetry
//   p / r    pause / resume                           x  stop everything
//   s <ms>   run <ms> of robot time, then pause       n  run to next node
//   b <0|1>  pause at every node                      v <pct>  slow motion
//   m <0-4>  mode: maze, line test, sensors, manual, motor test
//   d <L> <R>       manual drive (stops after MANUAL_TIMEOUT_MS without a new one)
//   j <L> <R> <ms>  manual jog for <ms>
//   z        restart the route
//   g <n>    fast capture of n sensor scans (G lines)
//   l <ms>   line must be gone this long before it counts as lost
//   i <L> <R>  reverse left / right motor (0|1)
//   a <0|1>  read each sensor twice (less crosstalk)
//   t <i> <v>  threshold of sensor i (8 = all)        w  save thresholds   e  back to config.h
// =====================================================================

void sendConfig() {
  Serial.print("C");
  for (int i = 0; i < 8; i++) { Serial.print(' '); Serial.print(threshold[i]); }
  for (int i = 0; i < 8; i++) { Serial.print(' '); Serial.print(BLACK_DIRECTION[i]); }
  Serial.print(' '); Serial.print(DRIVE_SPEED);
  Serial.print(' '); Serial.print(TURN_SPEED);
  Serial.print(' '); Serial.print(SEARCH_SPEED);
  for (int i = 0; i < 8; i++) { Serial.print(' '); Serial.print(THRESHOLD[i]); }   // config.h defaults
  Serial.print(' '); Serial.println(lineLostMs);
}

void sendTelemetry() {
  static unsigned long lastSend = 0;
  if (!telemetryOn) return;
  if (!Serial) { telemetryOn = false; return; }      // dashboard closed the port
  if (millis() - lastSend < TELEMETRY_MS) return;
  lastSend = millis();

  uint16_t flags = (paused ? 1 : 0) | (breakAtNodes ? 2 : 0) | (stepping ? 4 : 0)
                | (pauseAtNextNode ? 8 : 0) | (routeDone ? 16 : 0) | (goingHome ? 32 : 0)
                | (doubleRead ? 64 : 0) | (leftReversed ? 128 : 0) | (rightReversed ? 256 : 0);
  Serial.print("T "); Serial.print(millis());
  Serial.print(' ');  Serial.print(mode);
  Serial.print(' ');  Serial.print(flags);
  for (int i = 0; i < 8; i++) { Serial.print(' '); Serial.print(raw[i]); }
  Serial.print(' ');  printPattern();
  Serial.print(' ');  Serial.print(lastLineError);
  Serial.print(' ');  Serial.print(lastCorrection);
  Serial.print(' ');  Serial.print(requestedLeft);
  Serial.print(' ');  Serial.print(requestedRight);
  Serial.print(' ');  Serial.print(outputLeft);
  Serial.print(' ');  Serial.print(outputRight);
  Serial.print(' ');  Serial.print(speedScale);
  bool onRoute = mode == MODE_MAZE && pathIndex < pathLength;
  Serial.print(' ');  Serial.print(onRoute ? NODE_NAMES[path[pathIndex]] : "-");
  Serial.print(' ');  Serial.print(onRoute && pathIndex + 1 < pathLength ? NODE_NAMES[path[pathIndex + 1]] : "-");
  Serial.print(' ');  Serial.print(itemsCarried);
  Serial.print(' ');  Serial.print(pathIndex);
  Serial.print(' ');  Serial.print(pathLength);
  Serial.print(' ');  Serial.println(lineBlips);
}

// Read all sensors <n> times as fast as possible and print every scan:
//   G <micros> raw0..raw7      then  G end
// Motors stop during the capture. Used to find flicker / supply noise.
void captureSensors(int n) {
  stopMotors();
  Serial.println("G start");
  for (int k = 0; k < n; k++) {
    unsigned long t = micros();
    readSensors();
    Serial.print("G "); Serial.print(t);
    for (int i = 0; i < 8; i++) { Serial.print(' '); Serial.print(raw[i]); }
    Serial.println();
  }
  Serial.println("G end");
  applyMotors();
}

void runCommand(char* cmd) {
  char op = cmd[0];
  char* p = cmd + 1;
  long a = strtol(p, &p, 10);
  long b = strtol(p, &p, 10);
  long c = strtol(p, &p, 10);

  switch (op) {
    case '?': telemetryOn = true; sendConfig(); break;
    case 'q': telemetryOn = false; break;
    case 'p': paused = true; stepping = false; break;
    case 'r': paused = false; stepping = false; pauseAtNextNode = false; break;
    case 's': runMillis(); stepEnd = runMillis() + (a > 0 ? a : 150);
              stepping = true; paused = false; break;
    case 'n': pauseAtNextNode = true; stepping = false; paused = false; break;
    case 'b': breakAtNodes = a != 0; break;
    case 'v': speedScale = constrain(a, 5, 100); break;
    case 'm': if (a >= MODE_MAZE && a <= MODE_MOTOR_TEST && a != mode) {
                mode = a; abortRequested = true; resetPending = true;
                stepping = false; manualLeft = manualRight = 0;
                Serial.print("# mode "); Serial.println(mode);
              }
              break;
    case 'd': manualLeft = constrain(a, -255, 255); manualRight = constrain(b, -255, 255);
              manualUntil = millis() + MANUAL_TIMEOUT_MS; break;
    case 'j': manualLeft = constrain(a, -255, 255); manualRight = constrain(b, -255, 255);
              manualUntil = millis() + constrain(c, 0, 5000); break;
    case 'x': paused = true; stepping = false; pauseAtNextNode = false;
              manualLeft = manualRight = 0; manualUntil = millis(); break;
    case 'z': abortRequested = true; resetPending = true; Serial.println("# route reset"); break;
    case 't': if (a >= 0 && a <= 8) {                        // t <sensor 0-7, or 8 = all> <value>
                for (int i = 0; i < 8; i++) if (a == 8 || a == i) threshold[i] = constrain(b, 0, 1023);
                sendConfig();
              }
              break;
    case 'a': doubleRead = a != 0; break;
    case 'i': leftReversed = a != 0; rightReversed = b != 0;  // i <left 0|1> <right 0|1> reverse motors
              Serial.print("# reversed: left "); Serial.print(leftReversed);
              Serial.print(" right "); Serial.println(rightReversed); break;
    case 'g': captureSensors(constrain(a, 1, 2000)); break;    // g <n> fast sensor capture                     // a <0|1> sensor double read
    case 'l': lineLostMs = constrain(a, 0, 2000); sendConfig();             // l <ms> line-lost time
              Serial.print("# line lost after "); Serial.print(lineLostMs); Serial.println(" ms"); break;
    case 'w': saveThresholds(); Serial.println("# thresholds + line-lost time saved on the robot"); break;
    case 'e': for (int i = 0; i < 8; i++) threshold[i] = THRESHOLD[i];
              lineLostMs = LINE_LOST_MS;
              sendConfig(); Serial.println("# thresholds back to config.h (send w to save)"); break;
  }
  applyMotors();
}

void readCommands() {
  static char buf[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (len) { buf[len] = 0; runCommand(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = ch;
    }
  }
}

// Motors off until resumed; sensors and dashboard stay live
void holdWhilePaused() {
  applyMotors();
  while (paused && pausable() && !abortRequested) {
    runMillis();
    readCommands();
    readSensors();
    sendTelemetry();
  }
  runMillis();
  applyMotors();
}

// Call often, also from inside long moves (turns, waits)
void debugService() {
  readCommands();
  if (stepping && (long)(runMillis() - stepEnd) >= 0) { stepping = false; paused = true; }
  sendTelemetry();
  if (paused && pausable()) holdWhilePaused();
}

// Called when the robot reaches a node, after deciding what to do there
void nodeEvent() {
  if (pauseAtNextNode || breakAtNodes) {
    pauseAtNextNode = false;
    stepping = false;
    paused = true;
    Serial.println("# paused at node");
  }
  debugService();
}

void manualRun() {
  if ((long)(millis() - manualUntil) >= 0) manualLeft = manualRight = 0;
  readSensors();
  lineError();
  drive(manualLeft, manualRight);
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
  Serial.println(loadThresholds() ? "Settings: saved on robot" : "Settings: config.h");

  // 3 s to put the robot down: at the start, on the line, facing WEST.
  // The dashboard already works during this time.
  unsigned long t0 = millis();
  while (millis() - t0 < 3000 && !abortRequested) {
    readCommands();
    readSensors();
    sendTelemetry();
  }
}

void loop() {
  debugService();
  if (abortRequested) {            // mode change or route reset
    abortRequested = false;
    stopMotors();
    if (resetPending) { resetMaze(); resetPending = false; }
    return;
  }

  static unsigned long lastRun = 0;
  if (millis() - lastRun < LOOP_MS) return;
  lastRun = millis();

  switch (mode) {
    case MODE_MOTOR_TEST: motorTest();      break;
    case MODE_SENSORS:    printSensors();   break;
    case MODE_LINE_TEST:  testLineFollow(); break;
    case MODE_MANUAL:     manualRun();      break;
    default:              mazeRun();        break;
  }
}

