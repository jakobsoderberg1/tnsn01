

/*
  LABYRINTROBOT
  Arduino Leonardo + QTR-8A


  Vänster motor: 11, 3
  Höger motor:   5, 9


  A0 = vänster sensor
  A7 = höger sensor


  Vänsterhandsregeln:
  Vänster > Rakt > Höger > Vänd om
*/


// MOTORPINS
const int motor1pin1 = 11;
const int motor1pin2 = 3;
const int motor2pin1 = 5;
const int motor2pin2 = 9;


// SENSORPINS
const int sensorPins[8] = {
  A0, A1, A2, A3, A4, A5, A6, A7
};


// KALIBRERING
const int threshold[8] = {
  675, 552, 544, 732, 690, 532, 506, 614
};


const int blackDirection[8] = {
  -1, -1, 1, 1, 1, -1, -1, -1
};


bool black[8];


// HASTIGHETER
int baseSpeed = 125;
int turnSpeed = 150;


float Kp = 0.09;
float Kd = 0.35;


int lastError = 0;
int lastDirection = 1;


// LINJEAVBROTT
unsigned long lineLostTime = 0;
bool lineWasLost = false;


const unsigned long gapTime = 350;
const unsigned long maxTurnTime = 2000;


// SETUP
void setup() {
  Serial.begin(9600);


  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);


  stopMotors();
  delay(1000);
}


// HUVUDPROGRAM
void loop() {


  readSensors();


  if (isIntersection()) {
    handleIntersection();
    return;
  }


  int position = getLinePosition();


  if (position < 0) {
    handleLostLine();
    return;
  }


  lineWasLost = false;


  int error = position - 3500;


  if (error < 0) lastDirection = -1;
  if (error > 0) lastDirection = 1;


  int derivative = error - lastError;


  int correction =
    Kp * error + Kd * derivative;


  lastError = error;


  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;


  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);


  runMotors(leftSpeed, rightSpeed);
}


// LÄS SENSORER
void readSensors() {


  for (int i = 0; i < 8; i++) {


    int value = analogRead(sensorPins[i]);


    int signal =
      blackDirection[i] *
      (value - threshold[i]);


    black[i] = signal > 15;
  }
}


// LINJEPOSITION
int getLinePosition() {


  long weightedSum = 0;
  long total = 0;


  for (int i = 0; i < 8; i++) {


    int value = analogRead(sensorPins[i]);


    int signal =
      blackDirection[i] *
      (value - threshold[i]);


    if (signal < 15) signal = 0;


    weightedSum +=
      (long)signal * i * 1000;


    total += signal;
  }


  if (total < 30) return -1;


  return weightedSum / total;
}


// KORSNINGSDETEKTION
bool isIntersection() {


  int leftCount = 0;
  int rightCount = 0;


  for (int i = 0; i < 3; i++) {
    if (black[i]) leftCount++;
  }


  for (int i = 5; i < 8; i++) {
    if (black[i]) rightCount++;
  }


  return leftCount >= 2 && rightCount >= 2;
}


// KORSNINGSVAL
void handleIntersection() {


  runMotors(90, 90);
  delay(100);


  stopMotors();
  delay(100);


  readSensors();


  bool leftPath =
    black[0] || black[1] || black[2];


  bool rightPath =
    black[5] || black[6] || black[7];


  bool straightPath =
    black[3] || black[4];


  if (leftPath) {
    turnLeft();
  }
  else if (straightPath) {
    runMotors(baseSpeed, baseSpeed);
    delay(180);
  }
  else if (rightPath) {
    turnRight();
  }
  else {
    turnAround();
  }


  lastError = 0;
  lineWasLost = false;
}


// VÄNSTER
void turnLeft() {


  // Vrid på stället åt vänster
  runMotors(-150, 150);
  delay(600);


  // Leta efter linjen i mitten
  unsigned long startTime = millis();


  while (millis() - startTime < maxTurnTime) {


    readSensors();


    if (black[3] || black[4]) {
      break;
    }
  }


  runMotors(100, 100);
  delay(150);


  lastError = 0;
}


// HÖGER
void turnRight() {


  runMotors(150, -150);
  delay(600);


  unsigned long startTime = millis();


  while (millis() - startTime < maxTurnTime) {


    readSensors();


    if (black[3] || black[4]) {
      break;
    }
  }


  runMotors(100, 100);
  delay(150);


  lastError = 0;
}


// VÄND OM
void turnAround() {


  runMotors(150, -150);
  delay(900);


  unsigned long startTime = millis();


  while (millis() - startTime < maxTurnTime) {


    readSensors();


    if (black[3] || black[4]) {
      break;
    }
  }


  lastError = 0;
}


// LINJEAVBROTT
void handleLostLine() {


  if (!lineWasLost) {
    lineLostTime = millis();
    lineWasLost = true;
  }


  unsigned long lostDuration =
    millis() - lineLostTime;


  if (lostDuration < gapTime) {
    runMotors(120, 120);
  }
  else {
    if (lastDirection < 0) {
      runMotors(-90, 90);
    }
    else {
      runMotors(90, -90);
    }
  }
}


// MOTORSTYRNING
void runMotors(int leftSpeed, int rightSpeed) {


  setMotor(motor1pin1, motor1pin2, leftSpeed);
  setMotor(motor2pin1, motor2pin2, rightSpeed);
}


// MOTORRIKTNING
void setMotor(int pin1, int pin2, int speedValue) {


  speedValue = constrain(speedValue, -255, 255);


  // Omvänd riktning enligt motortestet
  if (speedValue > 0) {
    digitalWrite(pin1, LOW);
    analogWrite(pin2, speedValue);
  }
  else if (speedValue < 0) {
    analogWrite(pin1, -speedValue);
    digitalWrite(pin2, LOW);
  }
  else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}


// STOPP
void stopMotors() {
  runMotors(0, 0);
}



