#include <SPI.h>
#include <Pixy2.h> // Include the correct library for PixyCam 2
#include <Servo.h>

Pixy2 pixy;
Servo servoMotor;

int myPins[6] = {5, 6, 7, 8, 9, 10}; // ENA IN1 IN2 IN3 IN4 ENB
float deadZone = 0.15;
bool obstacleDetected = false;
int lastCubeColor = 0; // 1: Red, 2: Green

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  pixy.init();
  servoMotor.attach(4); // Attaching the servo motor to pin 4
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
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
}

float pixyCheck() {
  int blocks = pixy.ccc.getBlocks();
  if (blocks) {
    int signature = pixy.ccc.blocks[0].m_signature;
    if (signature == 1 || signature == 2) { // Red or green cube
      obstacleDetected = true;
      lastCubeColor = signature;
      return signature;
    }
  }
  return 0; // Return 0 if no blocks detected or other color detected
}

void moveRobot(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(myPins[1], LOW);
    digitalWrite(myPins[2], HIGH);
  } else {
    digitalWrite(myPins[1], HIGH);
    digitalWrite(myPins[2], LOW);
  }

  if (rightSpeed >= 0) {
    digitalWrite(myPins[3], LOW);
    digitalWrite(myPins[4], HIGH);
  } else {
    digitalWrite(myPins[3], HIGH);
    digitalWrite(myPins[4], LOW);
  }

  analogWrite(myPins[0], abs(leftSpeed));
  analogWrite(myPins[5], abs(rightSpeed));
}
