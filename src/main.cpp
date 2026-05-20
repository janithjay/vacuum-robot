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
const int echoFront  = 11;   // Front ultrasonic
const int echoLeft   = 12;   // Left ultrasonic
const int echoRight  = 13;   // Right ultrasonic

// ─────────────────────────────────────────────────────────────
// TUNABLE PARAMETERS — adjust these to suit your robot
// ─────────────────────────────────────────────────────────────
const int   DRIVE_SPEED        = 80;  // PWM for forward driving
const int   PIVOT_SPEED        = 80;   // PWM for pivot turns
const long  OBSTACLE_DIST_CM   = 7;   // Stop & decide if front < this (cm)
const long  SIDE_CLEAR_CM      = 5;   // Side is "blocked" if < this (cm)
const long  SONAR_TIMEOUT_US   = 20000;// pulseIn timeout (≈340 cm max range)

// ─────────────────────────────────────────────────────────────
// VARIABLES
// ─────────────────────────────────────────────────────────────
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

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

void moveForward(int speed)  { setLeftMotor(speed, 1);  setRightMotor(speed, 1);  }
void moveBackward(int speed) { setLeftMotor(speed, -1); setRightMotor(speed, -1); }

// Center-pivot — robot spins on its own center point
void pivotLeft(int speed)  { setLeftMotor(speed, -1); setRightMotor(speed, 1);  }
void pivotRight(int speed) { setLeftMotor(speed, 1);  setRightMotor(speed, -1); }

// Kill PWM first to prevent ghost current
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

// ─────────────────────────────────────────────────────────────
// BLOCKING PIVOT (gyro-controlled)
// Positive angle = left, negative = right
// Resets gyro to zero before each pivot — always accurate
// ─────────────────────────────────────────────────────────────
void blockingPivot(float targetAngle, int speed) {
  resetTracking();
  unsigned long startTime = millis();

  Serial.print("[PIVOT] Targeting "); Serial.print(targetAngle, 1); Serial.println("°");

  while (true) {
    updateGyroAngle();

    if (millis() - startTime > 5000) {
      Serial.println("[TIMEOUT] Pivot timeout!");
      stopMotors();
      return;
    }

    float angleDiff = targetAngle - gyroAngle;

    if (abs(angleDiff) < 5.0) {
      stopMotors();
      Serial.print("[PIVOT] Done. Final angle: "); Serial.print(gyroAngle, 1); Serial.println("°");
      return;
    }

    if (angleDiff > 0) pivotLeft(speed);
    else               pivotRight(speed);
  }
}

// ─────────────────────────────────────────────────────────────
// ULTRASONIC — shared trigger, individual echo pins
// Returns distance in cm. Returns 999 if no echo (clear path).
// A small delay between readings prevents cross-talk.
// ─────────────────────────────────────────────────────────────
long readSonar(int echoPin) {
  // Ensure trigger is clean before firing
  digitalWrite(sharedTrig, LOW);
  delayMicroseconds(4);
  digitalWrite(sharedTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sharedTrig, LOW);

  long duration = pulseIn(echoPin, HIGH, SONAR_TIMEOUT_US);
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// Read all three sensors with a small gap between each to avoid cross-talk
struct SonarReadings {
  long front;
  long left;
  long right;
};

SonarReadings readAllSonars() {
  SonarReadings s;
  s.front = readSonar(echoFront); delayMicroseconds(300);
  s.left  = readSonar(echoLeft);  delayMicroseconds(300);
  s.right = readSonar(echoRight); delayMicroseconds(300);
  return s;
}

// ─────────────────────────────────────────────────────────────
// OBSTACLE AVOIDANCE STATE MACHINE
//
// States:
//   FORWARD  — drive forward until front obstacle detected
//   DECIDE   — stop, read left/right, choose turn direction
//   TURN     — pivot 90° in chosen direction
//   BACKUP   — back up briefly if all three sides blocked
// ─────────────────────────────────────────────────────────────

enum RobotState { FORWARD, DECIDE, TURN_LEFT, TURN_RIGHT, BACKUP };

RobotState state = FORWARD;

void loop() {
  updateGyroAngle();

  SonarReadings s = readAllSonars();

  // Live debug every 500 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("[SONAR] F:"); Serial.print(s.front);
    Serial.print("cm L:");      Serial.print(s.left);
    Serial.print("cm R:");      Serial.print(s.right);
    Serial.print("cm | State: ");
    switch(state) {
      case FORWARD:    Serial.println("FORWARD");    break;
      case DECIDE:     Serial.println("DECIDE");     break;
      case TURN_LEFT:  Serial.println("TURN_LEFT");  break;
      case TURN_RIGHT: Serial.println("TURN_RIGHT"); break;
      case BACKUP:     Serial.println("BACKUP");     break;
    }
  }

  switch (state) {

    // ── FORWARD ──────────────────────────────────────────────
    // Drive forward. Transition to DECIDE if front is blocked.
    case FORWARD:
      moveForward(DRIVE_SPEED);
      if (s.front < OBSTACLE_DIST_CM) {
        stopMotors();
        Serial.println("\n[OBSTACLE] Front blocked! Deciding...");
        state = DECIDE;
      }
      break;

    // ── DECIDE ───────────────────────────────────────────────
    // Read sides and pick the clearest direction to turn.
    // If both sides are blocked too → BACKUP first.
    case DECIDE: {
      bool leftBlocked  = (s.left  < SIDE_CLEAR_CM);
      bool rightBlocked = (s.right < SIDE_CLEAR_CM);

      Serial.print("[DECIDE] Left "); Serial.print(leftBlocked  ? "BLOCKED" : "CLEAR");
      Serial.print(" | Right ");      Serial.println(rightBlocked ? "BLOCKED" : "CLEAR");

      if (!leftBlocked && !rightBlocked) {
        // Both clear — pick the side with more space
        if (s.left >= s.right) {
          Serial.println("[DECIDE] Both clear — turning LEFT (more space)");
          state = TURN_LEFT;
        } else {
          Serial.println("[DECIDE] Both clear — turning RIGHT (more space)");
          state = TURN_RIGHT;
        }
      } else if (!leftBlocked) {
        Serial.println("[DECIDE] Turning LEFT");
        state = TURN_LEFT;
      } else if (!rightBlocked) {
        Serial.println("[DECIDE] Turning RIGHT");
        state = TURN_RIGHT;
      } else {
        // All three sides blocked — back up and re-evaluate
        Serial.println("[DECIDE] All sides blocked! Backing up...");
        state = BACKUP;
      }
      break;
    }

    // ── TURN LEFT ────────────────────────────────────────────
    case TURN_LEFT:
      blockingPivot(90.0, PIVOT_SPEED);
      delay(200);
      state = FORWARD;
      break;

    // ── TURN RIGHT ───────────────────────────────────────────
    case TURN_RIGHT:
      blockingPivot(-90.0, PIVOT_SPEED);
      delay(200);
      state = FORWARD;
      break;

    // ── BACKUP ───────────────────────────────────────────────
    // Reverse for ~25 cm worth of ticks, then re-evaluate.
    case BACKUP: {
      Serial.println("[BACKUP] Reversing...");
      resetTracking();
      long targetTicks = (long)(25.0 * MEASURED_TICKS_PER_CM); // 25 cm back

      moveBackward(DRIVE_SPEED);
      while (leftTicks < targetTicks && rightTicks < targetTicks) {
        // Check front is now clear enough before full 25 cm
        long f = readSonar(echoFront);
        if (f > OBSTACLE_DIST_CM + 10) break;
        delay(10);
      }
      stopMotors();
      delay(300);

      // After backing up, turn 180° if completely stuck, else re-decide
      SonarReadings sr = readAllSonars();
      if (sr.front < OBSTACLE_DIST_CM && sr.left < SIDE_CLEAR_CM && sr.right < SIDE_CLEAR_CM) {
        Serial.println("[BACKUP] Still stuck — turning 180°");
        blockingPivot(180.0, PIVOT_SPEED);
      }

      state = FORWARD;
      break;
    }
  }
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

  pinMode(sharedTrig, OUTPUT);
  pinMode(echoFront,  INPUT);
  pinMode(echoLeft,   INPUT);
  pinMode(echoRight,  INPUT);
  digitalWrite(sharedTrig, LOW);

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
  Serial.println("OBSTACLE AVOIDANCE ACTIVE");
  Serial.print("Stop distance : "); Serial.print(OBSTACLE_DIST_CM); Serial.println("cm");
  Serial.print("Side clear    : "); Serial.print(SIDE_CLEAR_CM);    Serial.println("cm");
  Serial.print("Drive PWM     : "); Serial.println(DRIVE_SPEED);
  Serial.print("Pivot PWM     : "); Serial.println(PIVOT_SPEED);
  Serial.println("========================================\n");
  delay(500);
}
