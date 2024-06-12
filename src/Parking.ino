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
      if (pixy.ccc.blocks[i].m_signature == 5) { 
        parkCar();
      }
    }
  }

  moveForward();
  delay(100);
}

void parkCar() {
  stopMotors();
  Serial.println("Mașina este parcată.");
  while (1); 
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
