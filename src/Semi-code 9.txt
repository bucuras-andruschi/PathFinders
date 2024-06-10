#include <SPI.h>
#include <Pixy2.h> // Include the correct library for PixyCam 2
#include <Servo.h>

// TCS3200 color sensor pins
#define S0 9
#define S1 10
#define S2 11
#define S3 12
#define sensorOut 13

// Calibration
#define RED_LOW_PARA 40
#define RED_HIGH_PARA 200
#define GREEN_LOW_PARA 50
#define GREEN_HIGH_PARA 300
#define BLUE_LOW_PARA 50
#define BLUE_HIGH_PARA 300

int blue = 0;
int red = 0;
int green = 0;
int frequency = 0;

Pixy2 pixy;
Servo servoMotor;

int myPins[6] = {2, 3, 4, 5, 6, 7}; // ENA, IN1, IN2, IN3, IN4, ENB
float deadZone = 0.15;
bool obstacleDetected = false;
int lastCubeColor = 0; // 1: Red, 2: Green

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  // Initialize Pixy2 camera
  pixy.init();
  if (pixy.init() == 0) {
    Serial.println("Pixy2 initialization successful.");
  } else {
    Serial.println("Pixy2 initialization failed.");
  }

  // Attach the servo motor to pin 8
  servoMotor.attach(8);

  // Set motor control pins as outputs
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }

  // Set the color sensor pins as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void loop() {
  moveRobot(170, 170); // Adjust speed values according to your robot's dynamics for going forward
  pixyCheck();

  if (obstacleDetected) {
    if (lastCubeColor == 1) { // Red cube
      // Swerving right
      servoMotor.write(120); // Adjust servo position to turn right
      delay(500); // Adjust delay according to your control loop requirements for avoiding the obstacle

      // Continue moving forward and then correct to left
      while (pixyCheck() != 0); // Wait until the cube is not detected
      servoMotor.write(45); // Adjust servo position to turn left
      delay(500); // Adjust delay according to your control loop requirements for correcting

      // Return to center
      servoMotor.write(90); // Return to center position
    } else if (lastCubeColor == 2) { // Green cube
      // Swerving left
      servoMotor.write(45); // Adjust servo position to turn left
      delay(500); // Adjust delay according to your control loop requirements for avoiding the obstacle

      // Continue moving forward and then correct to right
      while (pixyCheck() != 0); // Wait until the cube is not detected
      servoMotor.write(120); // Adjust servo position to turn right
      delay(500); // Adjust delay according to your control loop requirements for correcting

      // Return to center
      servoMotor.write(90); // Return to center position
    }

    obstacleDetected = false;
    lastCubeColor = 0; // Reset last cube color after avoiding the obstacle
  }

  checkColorSensor(); // Check the color sensor and adjust servo accordingly
}

float pixyCheck() {
  int blocks = pixy.ccc.getBlocks();
  if (blocks) {
    int signature = pixy.ccc.blocks[0].m_signature;
    if (signature == 1 || signature == 2) { // Red or green cube
      obstacleDetected = true;
      lastCubeColor = signature;
      Serial.print("Detected cube color: ");
      if (signature == 1) {
        Serial.println("Red");
      } else {
        Serial.println("Green");
      }
      return signature;
    }
  }
  return 0; // Return 0 if no blocks detected or other color detected
}

void moveRobot(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(myPins[1], HIGH);
    digitalWrite(myPins[2], LOW);
  } else {
    digitalWrite(myPins[1], LOW);
    digitalWrite(myPins[2], HIGH);
  }

  if (rightSpeed >= 0) {
    digitalWrite(myPins[3], HIGH);
    digitalWrite(myPins[4], LOW);
  } else {
    digitalWrite(myPins[3], LOW);
    digitalWrite(myPins[4], HIGH);
  }

  analogWrite(myPins[0], abs(leftSpeed));
  analogWrite(myPins[5], abs(rightSpeed));
}

void checkColorSensor() {
  // Read red component
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, RED_HIGH_PARA, RED_LOW_PARA, 255, 0);
  red = frequency;

  // Read green component
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, GREEN_HIGH_PARA, GREEN_LOW_PARA, 255, 0);
  green = frequency;

  // Read blue component
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, BLUE_HIGH_PARA, BLUE_LOW_PARA, 255, 0);
  blue = frequency;

  // Print RGB values
  Serial.print("R= ");
  Serial.print(red);
  Serial.print(" G= ");
  Serial.print(green);
  Serial.print(" B= ");
  Serial.println(blue);

  // Check for white color
  if (green < -20 && blue < -20 && red < -20) {
    // Detected white color
    Serial.println("Detected color: White");
    servoMotor.write(90); // Keep servo at 90 degrees
  } else {
    // Detected any other color
    Serial.println("Detected color: Not white");
    servoMotor.write(120); // Move servo to 120 degrees
    delay(1800); // Wait for 300 ms
    servoMotor.write(90); // Return servo to 90 degrees
  }
}
