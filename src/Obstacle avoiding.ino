#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;
Servo myservo;

int motorLeftPin1 = 5;  
int motorLeftPin2 = 6;  
int motorRightPin1 = 9; 
int motorRightPin2 = 10;

void setup() {
  Serial.begin(115200);
  pixy.init();
  myservo.attach(3); 

  pinMode(motorLeftPin1, OUTPUT);
  pinMode(motorLeftPin2, OUTPUT);
  pinMode(motorRightPin1, OUTPUT);
  pinMode(motorRightPin2, OUTPUT);
}

void loop() {
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 3) { 
        avoidObstacle('L'); 
      } else if (pixy.ccc.blocks[i].m_signature == 4) { 
        avoidObstacle('R'); 
      }
    }
  }

  moveForward();
  delay(100); 
}

void avoidObstacle(char direction) {
  if (direction == 'L') {
    myservo.write(60); 
  } else if (direction == 'R') {
    myservo.write(120); 
  }
  delay(500); 
}

void moveForward() {
  digitalWrite(motorLeftPin1, HIGH);
  digitalWrite(motorLeftPin2, LOW);
  digitalWrite(motorRightPin1, HIGH);
  digitalWrite(motorRightPin2, LOW);
}

void stopMotors() {
  digitalWrite(motorLeftPin1, LOW);
  digitalWrite(motorLeftPin2, LOW);
  digitalWrite(motorRightPin1, LOW);
  digitalWrite(motorRightPin2, LOW);
}
