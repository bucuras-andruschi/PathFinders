#include <SPI.h>
#include <Pixy2.h> 
#include <Servo.h>

Pixy2 pixy;
Servo servoMotor;

int myPins[6] = {5, 6, 7, 8, 9, 10}; 
bool obstacleDetected = false;
int lastCubeColor = 0; // 1: Red, 2: Green

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  pixy.init();
  servoMotor.attach(4); 
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
  moveRobot(0, 0); 
}

void loop() {
  moveRobot(50, 50);
  pixyCheck();
  
  if (obstacleDetected) {
    
    moveRobot(0, 0);
    if (lastCubeColor == 1) { 
      // Turn right
      servoMotor.write(120); 
      delay(500); 

      
      moveRobot(50, 50);
      delay(500); 

      // Return to center
      moveRobot(0, 0);
      servoMotor.write(90); 
      delay(500); 
    } else if (lastCubeColor == 2) { 
     
      servoMotor.write(45); 
      delay(500); 

      
      moveRobot(50, 50);
      delay(500); 



      
      moveRobot(0, 0);
      servoMotor.write(90); 
      delay(500);
    }
    
    obstacleDetected = false;
    lastCubeColor = 0; 
  }

  delay(100); 
}

int pixyCheck() {
  int blocks = pixy.ccc.getBlocks();
  if (blocks) {
    int signature = pixy.ccc.blocks[0].m_signature;
    if (signature == 1 || signature == 2) { 
      obstacleDetected = true;
      lastCubeColor = signature;
      Serial.print("Detected color: ");
      Serial.println(signature == 1 ? "Red" : "Green");
      return signature;
    }
  }
  return 0; 
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
