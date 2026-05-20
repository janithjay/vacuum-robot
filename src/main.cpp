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
float         gyroAngle     = 0.0;
unsigned long lastGyroTime  = 0;
float         gyroBiasZ     = 0.0;
bool          gyroCalibrated = false;
float         gyroSign      = 1.0;

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
  // All LOW — motors fully off, no ghost current
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

// ── CENTER-PIVOT TURNS ────────────────────────────────────────
// Both wheels run at equal speed in OPPOSITE directions so the
// robot rotates exactly around its own center point.
void pivotLeft(int speed) {
  setLeftMotor(speed, -1);  // left wheel backward
  setRightMotor(speed, 1);  // right wheel forward
}

void pivotRight(int speed) {
  setLeftMotor(speed, 1);   // left wheel forward
  setRightMotor(speed, -1); // right wheel backward
}

// ── CLEAN STOP — no ghost current ────────────────────────────
// Sets ENA/ENB LOW first so H-bridge output goes to 0 V,
// then sets IN pins LOW. This prevents any residual movement.
void stopMotors() {
  // Kill PWM first — output drops to 0 immediately
  analogWrite(leftENA,  0);
  analogWrite(rightENB, 0);
  // Then set direction pins LOW
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

  gyroBiasZ     = sumZ / samples;
  gyroCalibrated = true;

  Serial.print("Gyro Z Bias: "); Serial.print(gyroBiasZ, 6); Serial.println(" rad/s");
  Serial.println("Calibration complete!\n");
}

void determineGyroSign() {
  Serial.println("\n========================================");
  Serial.println("GYRO SIGN CHECK");
  Serial.println("Pivot-left briefly to verify direction...");
  Serial.println("========================================\n");

  stopMotors();
  delay(200);  // allow any residual movement to settle

  gyroAngle    = 0.0;
  lastGyroTime = millis();

  // Use center-pivot so both wheels move equally — no net translation
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

  // CLEAN STOP — kill PWM before touching IN pins
  stopMotors();
  delay(300);  // let motors fully stop before reading result

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

  static unsigned long lastGyroDebug = 0;
  if (currentTime - lastGyroDebug >= 2000) {
    lastGyroDebug = currentTime;
    Serial.print("[GYRO] Raw Z: "); Serial.print(g.gyro.z, 6);
    Serial.print(" | Corrected: "); Serial.print(correctedZ, 6);
    Serial.print(" | Angle: "); Serial.print(gyroAngle, 2); Serial.println("°");
  }
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
// ROTATION CORE
// ─────────────────────────────────────────────────────────────

// Returns true when the robot has reached targetAngle (from a fresh zero).
// Uses center-pivot (both wheels) for true on-the-spot rotation.
bool rotateToAngle(float targetAngle, int speed, unsigned long& rotationStartTime) {
  unsigned long currentTime = millis();

  // Safety timeout
  if (currentTime - rotationStartTime > 5000) {
    Serial.println("[TIMEOUT] Rotation timeout! Stopping.");
    stopMotors();
    return true;
  }

  float angleDiff = targetAngle - gyroAngle;

  static unsigned long lastRotDebug = 0;
  if (currentTime - lastRotDebug >= 1000) {
    lastRotDebug = currentTime;
    Serial.print("[ROTATE] Target: "); Serial.print(targetAngle, 1);
    Serial.print("° | Current: "); Serial.print(gyroAngle, 1);
    Serial.print("° | Error: "); Serial.println(angleDiff, 1);
  }

  if (abs(angleDiff) < 5.0) {
    Serial.println("[SUCCESS] Target angle reached!");
    // Kill PWM first, then clear IN pins — eliminates ghost pulse
    stopMotors();
    return true;
  }

  // Center-pivot: both wheels, equal speed, opposite directions
  if (angleDiff > 0) pivotLeft(speed);
  else               pivotRight(speed);

  return false;
}

// Non-blocking wrapper used by the test state machine
bool performGyroRotation(float targetAngle, int speed,
                         unsigned long& rotationStartTime, bool& rotationStarted) {
  if (!rotationStarted) {
    resetTracking();
    rotationStartTime = millis();
    rotationStarted   = true;
  }
  return rotateToAngle(targetAngle, speed, rotationStartTime);
}

// ─────────────────────────────────────────────────────────────
// REUSABLE 90° LEFT PIVOT  ← use this everywhere for left turns
// ─────────────────────────────────────────────────────────────
// Blocking. Resets gyro to zero, targets 90°, PWM 80.
// Both wheels used — robot stays on the same center point.
// Safe stop at end — no ghost motor movement.
void performLeft90() {
  bool          rotationStarted   = false;
  bool          rotationCompleted = false;
  unsigned long rotationStartTime = 0;

  Serial.println("\n[TURN] Starting 90° left pivot...");

  while (!rotationCompleted) {
    updateGyroAngle();  // MUST be inside loop — drives the angle integration
    rotationCompleted = performGyroRotation(90.0, 80, rotationStartTime, rotationStarted);
  }

  Serial.println("[TURN] 90° left pivot complete.\n");
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
  Serial.println("Starting Robot Diagnostics...");

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
}

// ─────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────

void loop() {
  unsigned long currentTime = millis();

  updateGyroAngle();

  // Sequential sonar polling
  if (currentTime - prevSonarTime >= 40) {
    prevSonarTime = currentTime;
    distances[currentSonar] = readSonar(echoPins[currentSonar]);
    if (++currentSonar > 4) currentSonar = 0;
  }

  // ── ROTATION TEST STATE MACHINE ──────────────────────────
  static bool          rotationStarted   = false;
  static bool          rotationCompleted = false;
  static unsigned long rotationStartTime = 0;

  if (!rotationCompleted && currentTime > 6000) {
    if (!rotationStarted) {
      Serial.println("\n========================================");
      Serial.println("STARTING 90° LEFT PIVOT TEST");
      Serial.println("Motor PWM: 80 | Both wheels | Center pivot");
      Serial.println("========================================\n");
    }

    if (performGyroRotation(90.0, 80, rotationStartTime, rotationStarted)) {
      rotationCompleted = true;
      Serial.println("\n========================================");
      Serial.println("ROTATION TEST RESULTS");
      Serial.println("========================================");
      Serial.print("Target Angle:       90.0°\n");
      Serial.print("Final Gyro Angle:   "); Serial.print(gyroAngle, 2);                        Serial.println("°");
      Serial.print("Left  Ticks:        "); Serial.println(leftTicks);
      Serial.print("Right Ticks:        "); Serial.println(rightTicks);
      Serial.print("Tick Difference:    "); Serial.println(abs(leftTicks - rightTicks));
      Serial.print("Distance Left:      "); Serial.print(getDistanceFromTicks(leftTicks), 2);  Serial.println("cm");
      Serial.print("Distance Right:     "); Serial.print(getDistanceFromTicks(rightTicks), 2); Serial.println("cm");
      Serial.println("========================================\n");

      // ── To add a second identical left pivot, uncomment: ──
      // delay(1000);
      // performLeft90();
      // ───────────────────────────────────────────────────────
    }
  }

  // Live status
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint >= 1000) {
    lastPrint = currentTime;
    Serial.print("[TEST] Ticks L:");  Serial.print(leftTicks);
    Serial.print(" R:");              Serial.print(rightTicks);
    Serial.print(" | Gyro:");         Serial.print(gyroAngle, 1);
    Serial.print("° | ");
    if      (!rotationStarted)   Serial.println("Waiting for startup");
    else if (!rotationCompleted) Serial.println("Rotating");
    else                         Serial.println("Complete");
  }
}
