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

// Stop all motors
void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
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
}

void loop() {
  unsigned long currentTime = millis();

  // Sequential Sonar Polling
  if (currentTime - prevSonarTime >= 40) {
    prevSonarTime = currentTime;
    distances[currentSonar] = readSonar(echoPins[currentSonar]);
    currentSonar++;
    if (currentSonar > 4) currentSonar = 0; 
  }

  // Motor control sequence - test pattern
  static unsigned long motorTestTime = 0;
  static int motorTestState = 0;
  
  if (currentTime - motorTestTime >= 2000) {  // Change motor action every 2 seconds
    motorTestTime = currentTime;
    
    switch(motorTestState) {
      case 0:
        Serial.println("MOTOR TEST: Moving Forward...");
        moveForward(80);  // Slow speed
        break;
      case 1:
        Serial.println("MOTOR TEST: Moving Backward...");
        moveBackward(60);
        break;
      case 2:
        Serial.println("MOTOR TEST: Turning Left...");
        turnLeft(70);
        break;
      case 3:
        Serial.println("MOTOR TEST: Turning Right...");
        turnRight(70);
        break;
      case 4:
        Serial.println("MOTOR TEST: Stopped");
        stopMotors();
        break;
    }
    
    motorTestState++;
    if (motorTestState > 4) motorTestState = 0;
  }

  // Print Dashboard
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint >= 500) {
    lastPrint = currentTime;
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.println("\n--- SYSTEM DIAGNOSTIC ---");
    Serial.print("IMU Z Gyro: "); Serial.print(g.gyro.z); Serial.println(" rad/s");
    
    Serial.print("Encoders L: "); Serial.print(leftTicks); 
    Serial.print(" | R: "); Serial.println(rightTicks);
    
    Serial.print("Sonars cm F: "); Serial.print(distances[0]);
    Serial.print(" | L: "); Serial.print(distances[1]);
    Serial.print(" | R: "); Serial.print(distances[2]);
    Serial.print(" | DownL: "); Serial.print(distances[3]);
    Serial.print(" | DownR: "); Serial.println(distances[4]);
  }
}