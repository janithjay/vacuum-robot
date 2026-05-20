#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// PIN DEFINITIONS
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
const int echoDownL  = 14;
const int echoDownR  = 21;

// VARIABLES
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

unsigned long prevSonarTime = 0;
int  currentSonar = 0;
long distances[5] = {0, 0, 0, 0, 0};
int  echoPins[5]  = {echoFront, echoLeft, echoRight, echoDownL, echoDownR};

// Gyroscope
float         gyroAngle    = 0.0;
unsigned long lastGyroTime = 0;
float         gyroBiasZ    = 0.0;
float         gyroSign     = 1.0;

// Wheel / encoder parameters
const float WHEEL_DIAMETER_CM      = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.14159 * WHEEL_DIAMETER_CM;
const float WHEEL_BASE_CM          = 12.5;
const float WHEEL_OFFSET_BACK_CM   = 3.0;
const int   SLOTS_PER_REVOLUTION   = 20;
const float ENCODER_RESOLUTION_MM  = 0.01;
const float SLOT_WIDTH_MM          = 6.0;
const float MEASURED_TICKS_PER_CM  = 400.0;

// Test configuration
const int   PIVOT_SPEED      = 80;   // PWM for all pivots
const int   PAUSE_BETWEEN_MS = 800;  // pause between each pivot step (ms)

// INTERRUPTS
void IRAM_ATTR countLeft()  { leftTicks++; }
void IRAM_ATTR countRight() { rightTicks++; }

// ─────────────────────────────────────────────────────────────
// MOTOR CONTROL
// ─────────────────────────────────────────────────────────────

void setupMotors() {
  pinMode(leftENA,  OUTPUT); pinMode(leftIN1,  OUTPUT); pinMode(leftIN2,  OUTPUT);
  pinMode(rightENB, OUTPUT); pinMode(rightIN3, OUTPUT); pinMode(rightIN4, OUTPUT);
  digitalWrite(leftENA,  LOW); digitalWrite(leftIN1,  LOW); digitalWrite(leftIN2,  LOW);
  digitalWrite(rightENB, LOW); digitalWrite(rightIN3, LOW); digitalWrite(rightIN4, LOW);
}

void setLeftMotor(int speed, int direction) {
  analogWrite(leftENA, constrain(abs(speed), 0, 255));
  if      (direction > 0) { digitalWrite(leftIN1, HIGH); digitalWrite(leftIN2, LOW);  }
  else if (direction < 0) { digitalWrite(leftIN1, LOW);  digitalWrite(leftIN2, HIGH); }
  else                    { digitalWrite(leftIN1, LOW);  digitalWrite(leftIN2, LOW);  }
}

void setRightMotor(int speed, int direction) {
  analogWrite(rightENB, constrain(abs(speed), 0, 255));
  if      (direction > 0) { digitalWrite(rightIN3, HIGH); digitalWrite(rightIN4, LOW);  }
  else if (direction < 0) { digitalWrite(rightIN3, LOW);  digitalWrite(rightIN4, HIGH); }
  else                    { digitalWrite(rightIN3, LOW);  digitalWrite(rightIN4, LOW);  }
}

// Center-pivot: both wheels opposite directions — robot spins on its own center
void pivotLeft(int speed) {
  setLeftMotor(speed, -1);  // left backward
  setRightMotor(speed, 1);  // right forward
}

void pivotRight(int speed) {
  setLeftMotor(speed, 1);   // left forward
  setRightMotor(speed, -1); // right backward
}

// Kill PWM first, then direction pins — eliminates ghost current/movement
void stopMotors() {
  analogWrite(leftENA,  0);
  analogWrite(rightENB, 0);
  digitalWrite(leftIN1,  LOW); digitalWrite(leftIN2,  LOW);
  digitalWrite(rightIN3, LOW); digitalWrite(rightIN4, LOW);
}

void moveForward(int speed)  { setLeftMotor(speed, 1);  setRightMotor(speed, 1);  }
void moveBackward(int speed) { setLeftMotor(speed, -1); setRightMotor(speed, -1); }

// ─────────────────────────────────────────────────────────────
// GYROSCOPE
// ─────────────────────────────────────────────────────────────

void calibrateGyro() {
  Serial.println("\n========================================");
  Serial.println("GYRO CALIBRATION - Keep robot STILL!");
  Serial.println("Measuring for 3 seconds...");
  Serial.println("========================================\n");

  float sumZ = 0.0;
  int   samples = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 3000) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    samples++;
    delay(10);
  }

  gyroBiasZ = sumZ / samples;
  Serial.print("Gyro Z Bias: "); Serial.print(gyroBiasZ, 6); Serial.println(" rad/s");
  Serial.println("Calibration complete!\n");
}

void determineGyroSign() {
  Serial.println("\n========================================");
  Serial.println("GYRO SIGN CHECK — pivot-left briefly...");
  Serial.println("========================================\n");

  stopMotors();
  delay(200);

  gyroAngle    = 0.0;
  lastGyroTime = millis();

  pivotLeft(100);
  unsigned long startTime = millis();
  while (millis() - startTime < 300) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long now = millis();
    float dt = (now - lastGyroTime) / 1000.0;
    lastGyroTime = now;
    gyroAngle += ((g.gyro.z - gyroBiasZ) * 180.0 / 3.14159) * dt;
  }

  stopMotors();
  delay(300);

  Serial.print("Gyro angle after pivot-left: ");
  Serial.print(gyroAngle, 2); Serial.println("°");

  gyroSign = (gyroAngle < 0) ? -1.0 : 1.0;
  Serial.println(gyroSign < 0 ? "Gyro sign inverted." : "Gyro sign normal.");

  gyroAngle    = 0.0;
  lastGyroTime = millis();
}

void updateGyroAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  if (lastGyroTime == 0) { lastGyroTime = currentTime; return; }

  float dt         = (currentTime - lastGyroTime) / 1000.0;
  lastGyroTime     = currentTime;
  float correctedZ = (g.gyro.z - gyroBiasZ) * gyroSign;
  gyroAngle       += (correctedZ * 180.0 / 3.14159) * dt;
}

// ─────────────────────────────────────────────────────────────
// TRACKING HELPERS
// ─────────────────────────────────────────────────────────────

void resetTracking() {
  gyroAngle    = 0.0;
  lastGyroTime = millis();
  leftTicks    = 0;
  rightTicks   = 0;
}

float getDistanceFromTicks(long ticks) {
  return ticks / MEASURED_TICKS_PER_CM;
}

// ─────────────────────────────────────────────────────────────
// CORE BLOCKING PIVOT
// ─────────────────────────────────────────────────────────────
// Resets gyro to zero then pivots to targetAngle.
// Positive targetAngle = left, negative = right.
// Blocks until done or timeout (5 s).

void blockingPivot(float targetAngle, int speed) {
  resetTracking();
  unsigned long startTime = millis();

  while (true) {
    updateGyroAngle();

    // Safety timeout
    if (millis() - startTime > 5000) {
      Serial.println("[TIMEOUT] Pivot timeout!");
      stopMotors();
      return;
    }

    float angleDiff = targetAngle - gyroAngle;

    // Debug every 500 ms
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg >= 500) {
      lastDbg = millis();
      Serial.print("[PIVOT] Target: "); Serial.print(targetAngle, 1);
      Serial.print("° | Now: ");        Serial.print(gyroAngle, 1);
      Serial.print("° | Err: ");        Serial.println(angleDiff, 1);
    }

    if (abs(angleDiff) < 5.0) {
      stopMotors();
      Serial.println("[PIVOT] Done.");
      return;
    }

    if (angleDiff > 0) pivotLeft(speed);
    else               pivotRight(speed);
  }
}

// ─────────────────────────────────────────────────────────────
// SONAR
// ─────────────────────────────────────────────────────────────

long readSonar(int echoPin) {
  digitalWrite(sharedTrig, LOW);  delayMicroseconds(2);
  digitalWrite(sharedTrig, HIGH); delayMicroseconds(10);
  digitalWrite(sharedTrig, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return (duration == 0) ? 999 : duration * 0.034 / 2;
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Starting Robot...");

  setupMotors();
  Serial.println("Motors initialized!");

  pinMode(encLeftPin,  INPUT_PULLUP);
  pinMode(encRightPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encLeftPin),  countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(encRightPin), countRight, RISING);

  Serial.println("\n--- ENCODER CONFIGURATION ---");
  Serial.print("Wheel Diameter: ");             Serial.print(WHEEL_DIAMETER_CM);           Serial.println("cm");
  Serial.print("Wheel Circumference: ");        Serial.print(WHEEL_CIRCUMFERENCE_CM, 2);   Serial.println("cm");
  Serial.print("Wheel Base: ");                 Serial.print(WHEEL_BASE_CM);               Serial.println("cm");
  Serial.print("Wheel Offset Behind Center: "); Serial.print(WHEEL_OFFSET_BACK_CM);        Serial.println("cm");
  Serial.print("Slots per Revolution: ");       Serial.println(SLOTS_PER_REVOLUTION);
  Serial.print("Encoder Resolution: ");         Serial.print(ENCODER_RESOLUTION_MM);       Serial.println("mm");
  Serial.print("Slot Width: ");                 Serial.print(SLOT_WIDTH_MM);               Serial.println("mm");
  Serial.print("Measured Ticks per cm: ");      Serial.println(MEASURED_TICKS_PER_CM);
  Serial.print("Distance per tick: ");
  Serial.print(1.0 / MEASURED_TICKS_PER_CM, 5); Serial.println("cm\n");

  pinMode(sharedTrig, OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(echoPins[i], INPUT);

  Wire.begin(8, 9);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050! Check I2C wiring.");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Found!\n");

  delay(1000);
  calibrateGyro();
  determineGyroSign();

  Serial.println("\n========================================");
  Serial.println("TEST READY");
  Serial.println("Sequence: LEFT 90° → CENTER → RIGHT 90° → CENTER");
  Serial.println("Repeating until power off.");
  Serial.println("========================================\n");
  delay(1000);
}

// ─────────────────────────────────────────────────────────────
// LOOP  —  repeating pivot test
// ─────────────────────────────────────────────────────────────

void loop() {
  static int cycle = 1;

  Serial.print("\n=== CYCLE "); Serial.print(cycle); Serial.println(" ===");

  // ── STEP 1: Pivot 90° LEFT ───────────────────────────────
  Serial.println("[STEP 1] Pivot 90° LEFT");
  blockingPivot(90.0, PIVOT_SPEED);
  delay(PAUSE_BETWEEN_MS);

  // ── STEP 2: Return to center (pivot 90° RIGHT from current) ─
  Serial.println("[STEP 2] Return to center (90° RIGHT)");
  blockingPivot(-90.0, PIVOT_SPEED);
  delay(PAUSE_BETWEEN_MS);

  // ── STEP 3: Pivot 90° RIGHT ──────────────────────────────
  Serial.println("[STEP 3] Pivot 90° RIGHT");
  blockingPivot(-90.0, PIVOT_SPEED);
  delay(PAUSE_BETWEEN_MS);

  // ── STEP 4: Return to center (pivot 90° LEFT from current) ─
  Serial.println("[STEP 4] Return to center (90° LEFT)");
  blockingPivot(90.0, PIVOT_SPEED);
  delay(PAUSE_BETWEEN_MS);

  Serial.print("=== CYCLE "); Serial.print(cycle); Serial.println(" COMPLETE ===\n");
  cycle++;
}
