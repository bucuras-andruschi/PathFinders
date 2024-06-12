#include <SPI.h>
#include <Pixy2.h> 
#include <Servo.h>

Pixy2 pixy;
Servo servoMotor;

int myPins[6] = {2, 3, 4, 5, 6, 7}; 
float deadZone = 0.15;
bool obstacleDetected = false;
int lastCubeColor = 0; 

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  pixy.init();
  servoMotor.attach(8); 
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }
}

void loop() {
  moveRobot(170, 170); 
  pixyCheck();
  
  if (obstacleDetected) {
    if (lastCubeColor == 1) { 
   
      servoMotor.write(120);
      delay(500); 
      
      
      while (pixyCheck() != 0); 
      servoMotor.write(45); 
      delay(500); 
      
     
      servoMotor.write(90); 
    } else if (lastCubeColor == 2) { 
      
      servoMotor.write(45); 
      delay(500); 
      
     
      while (pixyCheck() != 0); 
      servoMotor.write(120); // Adjust servo position to turn right
      delay(500);
      
   
      servoMotor.write(90);
    }
    
    obstacleDetected = false;
    lastCubeColor = 0; 
  }
}

float pixyCheck() {
  int blocks = pixy.ccc.getBlocks();
  if (blocks) {
    int signature = pixy.ccc.blocks[0].m_signature;
    if (signature == 1 || signature == 2) { 
      obstacleDetected = true;
      lastCubeColor = signature;
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
