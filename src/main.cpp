#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// ─────────────────────────────────────────────────────────────
// PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────
const int leftENA  = 4;
const int leftIN1  = 5;
const int leftIN2  = 6;

const int rightENB = 7;
const int rightIN3 = 15;
const int rightIN4 = 16;

const int encLeftPin  = 17;
const int encRightPin = 18;

const int sharedTrig = 10;
const int echoFront  = 11;
const int echoLeft   = 12;
const int echoRight  = 13;

// ─────────────────────────────────────────────────────────────
// TUNABLE PARAMETERS
// ─────────────────────────────────────────────────────────────
const int   DRIVE_SPEED      = 90;
const int   PIVOT_SPEED      = 90;

const long  FRONT_STOP_CM    = 9;
const long  SIDE_CLEAR_CM    = 7;

const long  SONAR_TIMEOUT_US = 20000;

// ─────────────────────────────────────────────────────────────
// VARIABLES
// ─────────────────────────────────────────────────────────────
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

float gyroAngle = 0.0;
float gyroBiasZ = 0.0;
float gyroSign  = 1.0;

unsigned long lastGyroTime = 0;

// ─────────────────────────────────────────────────────────────
// ENCODERS
// ─────────────────────────────────────────────────────────────
void IRAM_ATTR countLeft() {
  leftTicks++;
}

void IRAM_ATTR countRight() {
  rightTicks++;
}

// ─────────────────────────────────────────────────────────────
// MOTOR CONTROL
// ─────────────────────────────────────────────────────────────
void setupMotors() {

  pinMode(leftENA, OUTPUT);
  pinMode(leftIN1, OUTPUT);
  pinMode(leftIN2, OUTPUT);

  pinMode(rightENB, OUTPUT);
  pinMode(rightIN3, OUTPUT);
  pinMode(rightIN4, OUTPUT);

  analogWrite(leftENA, 0);
  analogWrite(rightENB, 0);

  digitalWrite(leftIN1, LOW);
  digitalWrite(leftIN2, LOW);

  digitalWrite(rightIN3, LOW);
  digitalWrite(rightIN4, LOW);
}

void setLeftMotor(int speed, int direction) {

  analogWrite(leftENA, constrain(abs(speed), 0, 255));

  if (direction > 0) {
    digitalWrite(leftIN1, HIGH);
    digitalWrite(leftIN2, LOW);
  }
  else if (direction < 0) {
    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, HIGH);
  }
  else {
    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, LOW);
  }
}

void setRightMotor(int speed, int direction) {

  analogWrite(rightENB, constrain(abs(speed), 0, 255));

  if (direction > 0) {
    digitalWrite(rightIN3, HIGH);
    digitalWrite(rightIN4, LOW);
  }
  else if (direction < 0) {
    digitalWrite(rightIN3, LOW);
    digitalWrite(rightIN4, HIGH);
  }
  else {
    digitalWrite(rightIN3, LOW);
    digitalWrite(rightIN4, LOW);
  }
}

void moveForward(int speed) {
  setLeftMotor(speed, 1);
  setRightMotor(speed, 1);
}

void pivotLeft(int speed) {
  setLeftMotor(speed, -1);
  setRightMotor(speed, 1);
}

void pivotRight(int speed) {
  setLeftMotor(speed, 1);
  setRightMotor(speed, -1);
}

void stopMotors() {

  analogWrite(leftENA, 0);
  analogWrite(rightENB, 0);

  digitalWrite(leftIN1, LOW);
  digitalWrite(leftIN2, LOW);

  digitalWrite(rightIN3, LOW);
  digitalWrite(rightIN4, LOW);
}

// ─────────────────────────────────────────────────────────────
// MPU6050
// ─────────────────────────────────────────────────────────────
void calibrateGyro() {

  Serial.println("\nKeep robot still for calibration...");

  float sumZ = 0;
  int samples = 0;

  unsigned long start = millis();

  while (millis() - start < 3000) {

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumZ += g.gyro.z;
    samples++;

    delay(5);
  }

  gyroBiasZ = sumZ / samples;

  Serial.print("Gyro Bias Z = ");
  Serial.println(gyroBiasZ, 6);
}

void determineGyroSign() {

  Serial.println("Checking gyro direction...");

  gyroAngle = 0;
  lastGyroTime = millis();

  pivotLeft(80);

  unsigned long start = millis();

  while (millis() - start < 300) {

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();

    float dt = (now - lastGyroTime) / 1000.0;

    lastGyroTime = now;

    gyroAngle += ((g.gyro.z - gyroBiasZ) * 180.0 / PI) * dt;
  }

  stopMotors();

  delay(300);

  if (gyroAngle < 0) {
    gyroSign = -1.0;
  }
  else {
    gyroSign = 1.0;
  }

  Serial.println("Gyro sign configured.");

  gyroAngle = 0;
}

void resetGyroTracking() {

  gyroAngle = 0;
  lastGyroTime = millis();
}

void updateGyroAngle() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();

  if (lastGyroTime == 0) {
    lastGyroTime = now;
    return;
  }

  float dt = (now - lastGyroTime) / 1000.0;

  lastGyroTime = now;

  float correctedZ = (g.gyro.z - gyroBiasZ) * gyroSign;

  gyroAngle += (correctedZ * 180.0 / PI) * dt;
}

// ─────────────────────────────────────────────────────────────
// PRECISE 90° TURN
// ─────────────────────────────────────────────────────────────
void rotateDegrees(float targetAngle) {

  resetGyroTracking();

  bool turningLeft = (targetAngle > 0);

  Serial.print("Turning ");
  Serial.print(turningLeft ? "LEFT " : "RIGHT ");
  Serial.print(abs(targetAngle));
  Serial.println(" degrees");

  while (true) {

    updateGyroAngle();

    float error = targetAngle - gyroAngle;

    // STOP CONDITION
    if (abs(error) <= 3.0) {
      break;
    }

    if (turningLeft) {
      pivotLeft(PIVOT_SPEED);
    }
    else {
      pivotRight(PIVOT_SPEED);
    }

    delay(5);
  }

  stopMotors();

  delay(300);

  Serial.print("Final angle = ");
  Serial.println(gyroAngle);
}

// ─────────────────────────────────────────────────────────────
// ULTRASONIC
// ─────────────────────────────────────────────────────────────
long readSonar(int echoPin) {

  digitalWrite(sharedTrig, LOW);
  delayMicroseconds(4);

  digitalWrite(sharedTrig, HIGH);
  delayMicroseconds(10);

  digitalWrite(sharedTrig, LOW);

  long duration = pulseIn(echoPin, HIGH, SONAR_TIMEOUT_US);

  if (duration == 0) {
    return 999;
  }

  long distance = duration * 0.034 / 2;

  return distance;
}

struct Sonars {
  long front;
  long left;
  long right;
};

Sonars readAllSonars() {

  Sonars s;

  s.front = readSonar(echoFront);
  delay(30);

  s.left = readSonar(echoLeft);
  delay(30);

  s.right = readSonar(echoRight);
  delay(30);

  return s;
}

// ─────────────────────────────────────────────────────────────
// OBSTACLE AVOIDANCE
// ─────────────────────────────────────────────────────────────
void avoidObstacle() {

  stopMotors();

  Serial.println("\nObstacle detected!");
  Serial.println("Waiting 2 seconds...");

  // WAIT BEFORE DECIDING
  delay(2000);

  Sonars s = readAllSonars();

  Serial.print("Front : ");
  Serial.print(s.front);
  Serial.println(" cm");

  Serial.print("Left  : ");
  Serial.print(s.left);
  Serial.println(" cm");

  Serial.print("Right : ");
  Serial.print(s.right);
  Serial.println(" cm");

  bool leftClear  = (s.left  > SIDE_CLEAR_CM);
  bool rightClear = (s.right > SIDE_CLEAR_CM);

  // ─────────────────────────────────────
  // CHOOSE TURN DIRECTION
  // ─────────────────────────────────────

  // BOTH CLEAR
  if (leftClear && rightClear) {

    if (s.left >= s.right) {

      Serial.println("Turning LEFT");

      rotateDegrees(90);
    }
    else {

      Serial.println("Turning RIGHT");

      rotateDegrees(-90);
    }
  }

  // ONLY LEFT CLEAR
  else if (leftClear) {

    Serial.println("Left side free");

    rotateDegrees(90);
  }

  // ONLY RIGHT CLEAR
  else if (rightClear) {

    Serial.println("Right side free");

    rotateDegrees(-90);
  }

  // BOTH BLOCKED
  else {

    Serial.println("Both sides blocked");
    Serial.println("Turning LEFT 90°");

    rotateDegrees(90);
  }

  stopMotors();

  delay(300);
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {

  Serial.begin(115200);

  while (!Serial) {
    delay(10);
  }

  Serial.println("\nStarting Robot...");

  setupMotors();

  // ENCODERS
  pinMode(encLeftPin, INPUT_PULLUP);
  pinMode(encRightPin, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(encLeftPin),
    countLeft,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(encRightPin),
    countRight,
    RISING
  );

  // ULTRASONIC
  pinMode(sharedTrig, OUTPUT);

  pinMode(echoFront, INPUT);
  pinMode(echoLeft, INPUT);
  pinMode(echoRight, INPUT);

  digitalWrite(sharedTrig, LOW);

  // I2C
  Wire.begin(8, 9);

  // MPU6050
  if (!mpu.begin()) {

    Serial.println("MPU6050 not found!");

    while (1) {
      delay(10);
    }
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 connected");

  delay(1000);

  calibrateGyro();

  determineGyroSign();

  Serial.println("\nRobot Ready\n");
}

// ─────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────
void loop() {

  long frontDist = readSonar(echoFront);

  // STATUS PRINT
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 500) {

    lastPrint = millis();

    long leftDist  = readSonar(echoLeft);
    delay(20);

    long rightDist = readSonar(echoRight);

    Serial.print("F:");
    Serial.print(frontDist);

    Serial.print("  L:");
    Serial.print(leftDist);

    Serial.print("  R:");
    Serial.println(rightDist);
  }

  // OBSTACLE DETECTED
  if (frontDist <= FRONT_STOP_CM) {

    avoidObstacle();
  }

  // PATH CLEAR
  else {

    moveForward(DRIVE_SPEED);
  }

  delay(20);
}