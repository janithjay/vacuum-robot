#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// PIN DEFINITIONS
// Motor Control (L298N)
const int leftENA = 4;   // Left motor speed (PWM)
const int leftIN1 = 5;   // Left motor direction 1
const int leftIN2 = 6;   // Left motor direction 2
const int rightENB = 7;  // Right motor speed (PWM)
const int rightIN3 = 15; // Right motor direction 1
const int rightIN4 = 16; // Right motor direction 2

// Encoders
const int encLeftPin = 17;
const int encRightPin = 18;

// Ultrasonics
const int sharedTrig = 10;
const int echoFront = 11;
const int echoLeft = 12;
const int echoRight = 13;
const int echoDownL = 14;
const int echoDownR = 21;

// VARIABLES
volatile long leftTicks = 0;
volatile long rightTicks = 0;

unsigned long prevSonarTime = 0;
int currentSonar = 0; 
long distances[5] = {0, 0, 0, 0, 0}; 
int echoPins[5] = {echoFront, echoLeft, echoRight, echoDownL, echoDownR};

// Gyroscope tracking
float gyroAngle = 0.0;
unsigned long lastGyroTime = 0;
const float GYRO_CALIBRATION = 0.98;  // Complementary filter factor

// Gyro calibration (bias offset)
float gyroBiasZ = 0.0;
bool gyroCalibrated = false;
float gyroSign = 1.0;  // +1 if gyro reports positive for left turns

// Wheel parameters
const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = 3.14159 * WHEEL_DIAMETER_CM;  // ~20.42 cm

// Encoder specifications (HC-020K)
const float ENCODER_RESOLUTION_MM = 0.01;     // Resolution accuracy
const int SLOTS_PER_REVOLUTION = 20;          // Typical HC-020K has 20 slots (6mm slot width)
const float SLOT_WIDTH_MM = 6.0;              // Photoelectric slot width

// Measured encoder calibration
const float MEASURED_TICKS_PER_CM = 300.0;    // Actual observed sensor reading

// INTERRUPT FUNCTIONS
void IRAM_ATTR countLeft() { leftTicks++; }
void IRAM_ATTR countRight() { rightTicks++; }

// MOTOR CONTROL FUNCTIONS
void setupMotors() {
  // Configure motor control pins as outputs
  pinMode(leftENA, OUTPUT);
  pinMode(leftIN1, OUTPUT);
  pinMode(leftIN2, OUTPUT);
  pinMode(rightENB, OUTPUT);
  pinMode(rightIN3, OUTPUT);
  pinMode(rightIN4, OUTPUT);
  
  // Initialize all pins to LOW (motors off)
  digitalWrite(leftENA, 0);
  digitalWrite(leftIN1, 0);
  digitalWrite(leftIN2, 0);
  digitalWrite(rightENB, 0);
  digitalWrite(rightIN3, 0);
  digitalWrite(rightIN4, 0);
}

// Set left motor speed and direction
// speed: 0-255, direction: 1=forward, -1=backward, 0=stop
void setLeftMotor(int speed, int direction) {
  analogWrite(leftENA, abs(speed));
  
  if (direction > 0) {  // Forward
    digitalWrite(leftIN1, HIGH);
    digitalWrite(leftIN2, LOW);
  } 
  else if (direction < 0) {  // Backward
    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, HIGH);
  } 
  else {  // Stop
    digitalWrite(leftIN1, LOW);
    digitalWrite(leftIN2, LOW);
  }
}

// Set right motor speed and direction
void setRightMotor(int speed, int direction) {
  analogWrite(rightENB, abs(speed));
  
  if (direction > 0) {  // Forward
    digitalWrite(rightIN3, HIGH);
    digitalWrite(rightIN4, LOW);
  } 
  else if (direction < 0) {  // Backward
    digitalWrite(rightIN3, LOW);
    digitalWrite(rightIN4, HIGH);
  } 
  else {  // Stop
    digitalWrite(rightIN3, LOW);
    digitalWrite(rightIN4, LOW);
  }
}

// Move robot forward at given speed (0-255)
void moveForward(int speed) {
  setLeftMotor(speed, 1);
  setRightMotor(speed, 1);
}

// Move robot backward at given speed (0-255)
void moveBackward(int speed) {
  setLeftMotor(speed, -1);
  setRightMotor(speed, -1);
}

// Turn left (left motor slower/backward, right motor forward)
void turnLeft(int speed) {
  setLeftMotor(speed, -1);
  setRightMotor(speed, 1);
}

// Turn right (right motor slower/backward, left motor forward)
void turnRight(int speed) {
  setLeftMotor(speed, 1);
  setRightMotor(speed, -1);
}

// Stop all motors (coast)
void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}

// Brake all motors actively
void brakeMotors() {
  analogWrite(leftENA, 255);
  digitalWrite(leftIN1, HIGH);
  digitalWrite(leftIN2, HIGH);
  analogWrite(rightENB, 255);
  digitalWrite(rightIN3, HIGH);
  digitalWrite(rightIN4, HIGH);
}

// Calibrate gyroscope by measuring bias at startup
void calibrateGyro() {
  Serial.println("\n========================================");
  Serial.println("GYRO CALIBRATION - Keep robot STILL!");
  Serial.println("Measuring for 3 seconds...");
  Serial.println("========================================\n");
  
  float sumZ = 0.0;
  int samples = 0;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 3000) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    samples++;
    delay(10);
  }
  
  gyroBiasZ = sumZ / samples;
  gyroCalibrated = true;
  
  Serial.print("Gyro Z Bias: "); Serial.print(gyroBiasZ, 6);
  Serial.println(" rad/s");
  Serial.println("Calibration complete!\n");
}

void determineGyroSign() {
  Serial.println("\n========================================");
  Serial.println("GYRO SIGN CHECK");
  Serial.println("Turning left briefly to verify direction...");
  Serial.println("========================================\n");

  stopMotors();
  delay(100);
  gyroAngle = 0.0;
  lastGyroTime = millis();

  turnLeft(120);
  unsigned long startTime = millis();
  while (millis() - startTime < 300) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    unsigned long now = millis();
    float dt = (now - lastGyroTime) / 1000.0;
    lastGyroTime = now;
    float correctedZ = g.gyro.z - gyroBiasZ;
    gyroAngle += (correctedZ * 180.0 / 3.14159) * dt;
  }
  stopMotors();
  delay(200);

  Serial.print("Gyro angle after left turn: ");
  Serial.print(gyroAngle, 2);
  Serial.println("°");

  if (gyroAngle < 0) {
    gyroSign = -1.0;
    Serial.println("Gyro sign inverted to match left-turn direction.");
  } else {
    gyroSign = 1.0;
    Serial.println("Gyro sign normal.");
  }

  gyroAngle = 0.0;
  lastGyroTime = millis();
}

// Update gyroscope angle by integrating angular velocity (WITH bias correction)
void updateGyroAngle() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  unsigned long currentTime = millis();
  if (lastGyroTime == 0) {
    lastGyroTime = currentTime;
    return;
  }
  
  // Calculate time delta in seconds
  float deltaTime = (currentTime - lastGyroTime) / 1000.0;
  lastGyroTime = currentTime;
  
  // Subtract bias, correct orientation, and integrate angular velocity (Z-axis)
  float correctedZ = (g.gyro.z - gyroBiasZ) * gyroSign;
  float angleChange = (correctedZ * 180.0 / 3.14159) * deltaTime;
  gyroAngle += angleChange;
  
  // Debug: Print raw gyro values occasionally
  static unsigned long lastGyroDebug = 0;
  if (currentTime - lastGyroDebug >= 2000) {
    lastGyroDebug = currentTime;
    Serial.print("[GYRO] Raw Z: "); Serial.print(g.gyro.z, 6);
    Serial.print(" | Corrected: "); Serial.print(correctedZ, 6);
    Serial.print(" | Angle: "); Serial.print(gyroAngle, 2);
    Serial.println("°");
  }
}

// Rotate robot to target angle (positive = counterclockwise, negative = clockwise)
// Returns true when rotation is complete
bool rotateToAngle(float targetAngle, int speed, unsigned long& rotationStartTime);

void resetTracking();

bool rotateToAngle(float targetAngle, int speed, unsigned long& rotationStartTime) {
  unsigned long currentTime = millis();
  
  // Timeout after 5 seconds to prevent infinite rotation
  if (currentTime - rotationStartTime > 5000) {
    Serial.println("[TIMEOUT] Rotation timeout! Stopping motors.");
    stopMotors();
    return true;
  }
  
  float angleDiff = targetAngle - gyroAngle;
  
  // Print debug info every 1 second
  static unsigned long lastRotDebug = 0;
  if (currentTime - lastRotDebug >= 1000) {
    lastRotDebug = currentTime;
    Serial.print("[ROTATE] Target: "); Serial.print(targetAngle, 1);
    Serial.print("° | Current: "); Serial.print(gyroAngle, 1);
    Serial.print("° | Error: "); Serial.println(angleDiff, 1);
  }
  
  // Allow 5-degree tolerance for completion
  if (abs(angleDiff) < 5.0) {
    Serial.println("[SUCCESS] Target angle reached! Braking.");
    brakeMotors();
    delay(100);
    stopMotors();
    return true;
  }
  
  // Slow down as we approach the target angle to prevent overshoot
  int controlledSpeed = speed;
  if (abs(angleDiff) < 30.0) controlledSpeed = min(controlledSpeed, 110);
  if (abs(angleDiff) < 20.0) controlledSpeed = min(controlledSpeed, 85);
  if (abs(angleDiff) < 10.0) controlledSpeed = min(controlledSpeed, 70);
  
  // Ensure minimum spin power
  controlledSpeed = max(controlledSpeed, 70);
  
  // Rotate left if target is greater than current
  if (angleDiff > 0) {
    turnLeft(controlledSpeed);
  } 
  // Rotate right if target is less than current
  else {
    turnRight(controlledSpeed);
  }
  
  return false;
}

// Execute a single gyro-based rotation to the target angle
bool performGyroRotation(float targetAngle, int speed, unsigned long &rotationStartTime, bool &rotationStarted) {
  if (!rotationStarted) {
    resetTracking();
    rotationStartTime = millis();
    rotationStarted = true;
  }
  return rotateToAngle(targetAngle, speed, rotationStartTime);
}

// Reset gyro angle and encoders for new test
void resetTracking() {
  gyroAngle = 0.0;
  lastGyroTime = millis();
  leftTicks = 0;
  rightTicks = 0;
}

// Calculate distance traveled based on encoder ticks
// Uses measured values from your sensor: 300 ticks per cm
float getDistanceFromTicks(long ticks) {
  return ticks / MEASURED_TICKS_PER_CM;
}

// Function to read one sonar safely
long readSonar(int echoPin) {
  digitalWrite(sharedTrig, LOW); delayMicroseconds(2);
  digitalWrite(sharedTrig, HIGH); delayMicroseconds(10);
  digitalWrite(sharedTrig, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); 
  
  Serial.println("Starting Robot Diagnostics...");

  // Setup Motors
  setupMotors();
  Serial.println("Motors initialized!");
  
  // Setup Encoders
  pinMode(encLeftPin, INPUT_PULLUP);
  pinMode(encRightPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encLeftPin), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(encRightPin), countRight, RISING);

  // Display encoder configuration
  Serial.println("\n--- ENCODER CONFIGURATION ---");
  Serial.print("Wheel Diameter: "); Serial.print(WHEEL_DIAMETER_CM); Serial.println("cm");
  Serial.print("Wheel Circumference: "); Serial.print(WHEEL_CIRCUMFERENCE_CM, 2); Serial.println("cm");
  Serial.print("Slots per Revolution: "); Serial.println(SLOTS_PER_REVOLUTION);
  Serial.print("Encoder Resolution: "); Serial.print(ENCODER_RESOLUTION_MM); Serial.println("mm");
  Serial.print("Slot Width: "); Serial.print(SLOT_WIDTH_MM); Serial.println("mm");
  Serial.print("Measured Ticks per cm: "); Serial.println(MEASURED_TICKS_PER_CM);
  Serial.print("Distance per tick: "); 
  Serial.print(1.0 / MEASURED_TICKS_PER_CM, 5); Serial.println("cm\n");

  // Setup Ultrasonics
  pinMode(sharedTrig, OUTPUT);
  for(int i=0; i<5; i++) { pinMode(echoPins[i], INPUT); }

  // Setup MPU6050
  Wire.begin(8, 9); 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050! Check I2C wiring.");
    while (1) { delay(10); }
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 Found!");
  
  // Calibrate gyroscope
  delay(1000);
  calibrateGyro();
  determineGyroSign();
}

void loop() {
  unsigned long currentTime = millis();

  // Update gyroscope angle continuously
  updateGyroAngle();

  // Sequential Sonar Polling
  if (currentTime - prevSonarTime >= 40) {
    prevSonarTime = currentTime;
    distances[currentSonar] = readSonar(echoPins[currentSonar]);
    currentSonar++;
    if (currentSonar > 4) currentSonar = 0; 
  }

  // MOTOR ROTATION TEST - With sufficient power (PWM 150, min 70 verified)
  static bool rotationStarted = false;
  static bool rotationCompleted = false;
  static unsigned long rotationStartTime = 0;

  if (!rotationCompleted) {
    if (currentTime > 6000) {  // Wait 6 seconds for gyro calibration
      if (!rotationStarted) {
        Serial.println("\n========================================");
        Serial.println("STARTING 90° LEFT ROTATION TEST");
        Serial.println("Motor PWM: 150 (minimum 70 verified)");
        Serial.println("========================================\n");
      }

      if (performGyroRotation(90.0, 150, rotationStartTime, rotationStarted)) {
        rotationCompleted = true;
        Serial.println("\n========================================");
        Serial.println("ROTATION TEST RESULTS");
        Serial.println("========================================");
        Serial.print("Target Angle: 90.0°\n");
        Serial.print("Final Gyro Angle: "); Serial.print(gyroAngle, 2); Serial.println("°");
        Serial.print("Left Encoder Ticks: "); Serial.println(leftTicks);
        Serial.print("Right Encoder Ticks: "); Serial.println(rightTicks);
        Serial.print("Tick Difference: "); Serial.println(abs(leftTicks - rightTicks));
        Serial.print("Distance Left: "); Serial.print(getDistanceFromTicks(leftTicks), 2); Serial.println("cm");
        Serial.print("Distance Right: "); Serial.print(getDistanceFromTicks(rightTicks), 2); Serial.println("cm");
        Serial.println("========================================\n");
      }
    }
  }

  // Print Live Status
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint >= 1000) {
    lastPrint = currentTime;
    
    Serial.print("[ROTATION TEST] ");
    Serial.print("Ticks - L: "); Serial.print(leftTicks);
    Serial.print(" | R: "); Serial.print(rightTicks);
    Serial.print(" | Gyro: "); Serial.print(gyroAngle, 1);
    Serial.print("° | Status: ");
    if (!rotationStarted) Serial.println("Waiting for startup");
    else if (!rotationCompleted) Serial.println("Rotating");
    else Serial.println("Complete");
  }
}