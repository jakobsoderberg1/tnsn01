/*
  SERVO TEST
  Arduino Leonardo + 2 continuous-rotation servos

  Servo A: pin 12
  Servo B: pin 13

  Wiring:
    - Signal (orange/yellow) -> pin 12 / pin 13
    - Power (red)            -> separate 5-6 V supply (+)
    - Ground (brown/black)   -> supply (-) AND Arduino GND (shared ground!)

  Continuous servo: write(90) = stop, >90 = one direction, <90 = other.
  If a servo creeps during "stop", its center point is slightly off
  (calibration, not wiring).

  Open Serial Monitor at 9600 baud to see which step is running.
*/

#include <Servo.h>

const int PIN_A = 12;
const int PIN_B = 13;

const int STOP_POS = 90;
const int SPEED_OFFSET = 40;          // offset from STOP_POS (0-90)
const unsigned long STEP_MS = 2000;

Servo servoA;
Servo servoB;

void step(const char *label, int a, int b) {
  Serial.println(label);
  servoA.write(a);
  servoB.write(b);
  delay(STEP_MS);
}

void setup() {
  Serial.begin(9600);
  // Leonardo: wait up to 3 s for Serial Monitor, then run anyway
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) ;

  servoA.attach(PIN_A);
  servoB.attach(PIN_B);
  servoA.write(STOP_POS);
  servoB.write(STOP_POS);

  Serial.println("Servo test starting...");
  delay(1000);
}

void loop() {
  Serial.println("--- Pin 12 only ---");
  step("Pin 12: forward", STOP_POS + SPEED_OFFSET, STOP_POS);
  step("Pin 12: stop",    STOP_POS,         STOP_POS);
  step("Pin 12: reverse", STOP_POS - SPEED_OFFSET, STOP_POS);
  step("Pin 12: stop",    STOP_POS,         STOP_POS);

  Serial.println("--- Pin 13 only ---");
  step("Pin 13: forward", STOP_POS, STOP_POS + SPEED_OFFSET);
  step("Pin 13: stop",    STOP_POS, STOP_POS);
  step("Pin 13: reverse", STOP_POS, STOP_POS - SPEED_OFFSET);
  step("Pin 13: stop",    STOP_POS, STOP_POS);

  Serial.println("--- Both (same signal; mirrored wheels will spin in place) ---");
  step("Both: forward", STOP_POS + SPEED_OFFSET, STOP_POS + SPEED_OFFSET);
  step("Both: stop",    STOP_POS,         STOP_POS);

  Serial.println();
}
