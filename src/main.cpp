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
const int   DRIVE_SPEED      = 90;  // PWM for forward driving
const int   PIVOT_SPEED      = 90;   // PWM for pivot turns
const long  OBSTACLE_DIST_CM = 9;   // stop if front closer than this
const long  SIDE_CLEAR_CM    = 7;   // side "blocked" if closer than this
const long  SONAR_TIMEOUT_US = 20000;

// ─────────────────────────────────────────────────────────────
// VARIABLES
// ─────────────────────────────────────────────────────────────
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

float         gyroAngle    = 0.0;
unsigned long lastGyroTime = 0;
float         gyroBiasZ    = 0.0;
float         gyroSign     = 1.0;

const float WHEEL_DIAMETER_CM      = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.14159 * WHEEL_DIAMETER_CM;
const float WHEEL_BASE_CM          = 12.5;
const float WHEEL_OFFSET_BACK_CM   = 3.0;
const int   SLOTS_PER_REVOLUTION   = 20;
const float ENCODER_RESOLUTION_MM  = 0.01;
const float SLOT_WIDTH_MM          = 6.0;
const float MEASURED_TICKS_PER_CM  = 400.0;

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

void moveForward(int speed) { setLeftMotor(speed, 1); setRightMotor(speed, 1); }
void pivotLeft(int speed)   { setLeftMotor(speed, -1); setRightMotor(speed, 1);  }
void pivotRight(int speed)  { setLeftMotor(speed, 1);  setRightMotor(speed, -1); }

// Kill PWM first — no ghost current
void stopMotors() {
  analogWrite(leftENA,  0);
  analogWrite(rightENB, 0);
  digitalWrite(leftIN1,  LOW); digitalWrite(leftIN2,  LOW);
  digitalWrite(rightIN3, LOW); digitalWrite(rightIN4, LOW);
}

// ─────────────────────────────────────────────────────────────
// GYROSCOPE
// ─────────────────────────────────────────────────────────────

void calibrateGyro() {
  Serial.println("\n========================================");
  Serial.println("GYRO CALIBRATION - Keep robot STILL!");
  Serial.println("Measuring for 3 seconds...");
  Serial.println("========================================\n");

  float sumZ = 0.0; int samples = 0;
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z; samples++; delay(10);
  }
  gyroBiasZ = sumZ / samples;
  Serial.print("Gyro Z Bias: "); Serial.print(gyroBiasZ, 6); Serial.println(" rad/s");
  Serial.println("Calibration complete!\n");
}

void determineGyroSign() {
  Serial.println("GYRO SIGN CHECK — pivot-left briefly...");
  stopMotors(); delay(200);
  gyroAngle = 0.0; lastGyroTime = millis();

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
  stopMotors(); delay(300);

  gyroSign = (gyroAngle < 0) ? -1.0 : 1.0;
  Serial.println(gyroSign < 0 ? "Gyro sign inverted." : "Gyro sign normal.");
  gyroAngle = 0.0; lastGyroTime = millis();
}

void updateGyroAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  unsigned long currentTime = millis();
  if (lastGyroTime == 0) { lastGyroTime = currentTime; return; }
  float dt = (currentTime - lastGyroTime) / 1000.0;
  lastGyroTime = currentTime;
  float correctedZ = (g.gyro.z - gyroBiasZ) * gyroSign;
  gyroAngle += (correctedZ * 180.0 / 3.14159) * dt;
}

void resetTracking() {
  gyroAngle = 0.0; lastGyroTime = millis();
  leftTicks = 0;   rightTicks = 0;
}

// ─────────────────────────────────────────────────────────────
// BLOCKING PIVOT (gyro-controlled)
// Positive angle = left, negative = right
// ─────────────────────────────────────────────────────────────
void blockingPivot(float targetAngle, int speed) {
  resetTracking();
  unsigned long startTime = millis();
  Serial.print("[PIVOT] "); Serial.print(targetAngle > 0 ? "LEFT " : "RIGHT ");
  Serial.print(abs(targetAngle), 0); Serial.println("°");

  while (true) {
    updateGyroAngle();
    if (millis() - startTime > 5000) { Serial.println("[TIMEOUT]"); stopMotors(); return; }
    float err = targetAngle - gyroAngle;
    if (abs(err) < 5.0) { stopMotors(); return; }
    if (err > 0) pivotLeft(speed); else pivotRight(speed);
  }
}

// ─────────────────────────────────────────────────────────────
// ULTRASONIC
// ─────────────────────────────────────────────────────────────
long readSonar(int echoPin) {
  digitalWrite(sharedTrig, LOW);  delayMicroseconds(4);
  digitalWrite(sharedTrig, HIGH); delayMicroseconds(10);
  digitalWrite(sharedTrig, LOW);
  long duration = pulseIn(echoPin, HIGH, SONAR_TIMEOUT_US);
  return (duration == 0) ? 999 : duration * 0.034 / 2;
}

struct Sonars { long front, left, right; };

Sonars readAllSonars() {
  Sonars s;
  s.front = readSonar(echoFront); delayMicroseconds(300);
  s.left  = readSonar(echoLeft);  delayMicroseconds(300);
  s.right = readSonar(echoRight); delayMicroseconds(300);
  return s;
}

// ─────────────────────────────────────────────────────────────
// FIND FREE WAY  — no backward movement ever
//
// Called immediately when front obstacle detected.
// Robot is stopped. Logic:
//
//   1. Read all 3 sensors
//   2. If left clear  → pivot 90° left,  check front again
//   3. If right clear → pivot 90° right, check front again
//   4. If both clear  → pick side with more space
//   5. If all 3 blocked → keep pivoting LEFT in 45° steps
//      until front is clear (full 360° scan if needed)
//      Robot never moves backward — only rotates in place.
// ─────────────────────────────────────────────────────────────
void findFreeWay() {
  Serial.println("\n[SCAN] Obstacle detected — finding free way...");

  Sonars s = readAllSonars();
  Serial.print("[SCAN] F:"); Serial.print(s.front);
  Serial.print(" L:");       Serial.print(s.left);
  Serial.print(" R:");       Serial.print(s.right); Serial.println("cm");

  // ── Case 1: front already cleared (e.g. moving obstacle) ─
  if (s.front >= OBSTACLE_DIST_CM) {
    Serial.println("[SCAN] Front cleared — no turn needed.");
    return;
  }

  // ── Case 2: one or both sides open ───────────────────────
  bool leftClear  = (s.left  >= SIDE_CLEAR_CM);
  bool rightClear = (s.right >= SIDE_CLEAR_CM);

  if (leftClear || rightClear) {
    if (leftClear && rightClear) {
      // Both open — turn toward side with more space
      if (s.left >= s.right) {
        Serial.println("[SCAN] Both clear — turning LEFT (more space)");
        blockingPivot(90.0, PIVOT_SPEED);
      } else {
        Serial.println("[SCAN] Both clear — turning RIGHT (more space)");
        blockingPivot(-90.0, PIVOT_SPEED);
      }
    } else if (leftClear) {
      Serial.println("[SCAN] LEFT clear — turning LEFT 90°");
      blockingPivot(90.0, PIVOT_SPEED);
    } else {
      Serial.println("[SCAN] RIGHT clear — turning RIGHT 90°");
      blockingPivot(-90.0, PIVOT_SPEED);
    }
    delay(150);
    return;
  }

  // ── Case 3: all 3 blocked — rotate in 45° steps until ───
  //    front sonar finds a clear path. No backward movement.
  Serial.println("[SCAN] All blocked — rotating in 45° steps to find gap...");

  int totalRotated = 0;
  while (totalRotated < 360) {
    blockingPivot(45.0, PIVOT_SPEED);   // always pivot left in small steps
    totalRotated += 45;
    delay(150);

    long frontNow = readSonar(echoFront);
    Serial.print("[SCAN] Rotated "); Serial.print(totalRotated);
    Serial.print("° | Front: "); Serial.print(frontNow); Serial.println("cm");

    if (frontNow >= OBSTACLE_DIST_CM) {
      Serial.println("[SCAN] Gap found — resuming forward.");
      return;
    }
  }

  // Completed full 360° — front still blocked (very tight space)
  // Stay in place; next loop iteration will re-trigger findFreeWay
  Serial.println("[SCAN] Full 360° scanned. Will re-evaluate.");
}

// ─────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Starting Robot...");

  setupMotors();

  pinMode(encLeftPin,  INPUT_PULLUP);
  pinMode(encRightPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encLeftPin),  countLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(encRightPin), countRight, RISING);

  pinMode(sharedTrig, OUTPUT);
  pinMode(echoFront,  INPUT);
  pinMode(echoLeft,   INPUT);
  pinMode(echoRight,  INPUT);
  digitalWrite(sharedTrig, LOW);

  Wire.begin(8, 9);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Found!\n");

  delay(1000);
  calibrateGyro();
  determineGyroSign();

  Serial.println("\n========================================");
  Serial.println("OBSTACLE AVOIDANCE ACTIVE");
  Serial.println("Mode: FORWARD + ROTATE ONLY (no backward)");
  Serial.print("Obstacle trigger : "); Serial.print(OBSTACLE_DIST_CM); Serial.println("cm");
  Serial.print("Side clear min   : "); Serial.print(SIDE_CLEAR_CM);    Serial.println("cm");
  Serial.print("Drive PWM        : "); Serial.println(DRIVE_SPEED);
  Serial.print("Pivot PWM        : "); Serial.println(PIVOT_SPEED);
  Serial.println("========================================\n");
  delay(500);
}

// ─────────────────────────────────────────────────────────────
// LOOP
//
//   Every iteration:
//     1. Read front sonar
//     2. If clear  → drive forward
//     3. If blocked → stop + findFreeWay() (rotate only)
//        → loop resumes, drives forward in new direction
//
//   Robot NEVER moves backward.
// ─────────────────────────────────────────────────────────────
void loop() {
  updateGyroAngle();

  long frontDist = readSonar(echoFront);

  // Live status every 500 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    long l = readSonar(echoLeft);  delayMicroseconds(300);
    long r = readSonar(echoRight);
    Serial.print("[LIVE] F:"); Serial.print(frontDist);
    Serial.print("cm L:");     Serial.print(l);
    Serial.print("cm R:");     Serial.print(r); Serial.println("cm");
  }

  if (frontDist < OBSTACLE_DIST_CM) {
    // Obstacle — stop and rotate to find free way
    stopMotors();
    findFreeWay();
    // Loop continues → moveForward() called next iteration
  } else {
    // Clear path — drive forward
    moveForward(DRIVE_SPEED);
  }
}
