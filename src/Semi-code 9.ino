#include <SPI.h>
#include <Pixy2.h> 
#include <Servo.h>


#define S0 9
#define S1 10
#define S2 11
#define S3 12
#define sensorOut 13


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

int myPins[6] = {2, 3, 4, 5, 6, 7}; 
float deadZone = 0.15;
bool obstacleDetected = false;
int lastCubeColor = 0; 

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

 
  pixy.init();
  if (pixy.init() == 0) {
    Serial.println("Pixy2 initialization successful.");
  } else {
    Serial.println("Pixy2 initialization failed.");
  }

 
  servoMotor.attach(8);

  
  for (int i = 0; i < 6; i++) {
    pinMode(myPins[i], OUTPUT);
  }

 
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
      servoMotor.write(120); 
      delay(500); 

      
      servoMotor.write(90); 
    }

    obstacleDetected = false;
    lastCubeColor = 0; 
  }

  checkColorSensor(); 
}

float pixyCheck() {
  int blocks = pixy.ccc.getBlocks();
  if (blocks) {
    int signature = pixy.ccc.blocks[0].m_signature;
    if (signature == 1 || signature == 2) {
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
  return 0; 
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
 
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, RED_HIGH_PARA, RED_LOW_PARA, 255, 0);
  red = frequency;

 
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, GREEN_HIGH_PARA, GREEN_LOW_PARA, 255, 0);
  green = frequency;

  
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, BLUE_HIGH_PARA, BLUE_LOW_PARA, 255, 0);
  blue = frequency;

  
  Serial.print("R= ");
  Serial.print(red);
  Serial.print(" G= ");
  Serial.print(green);
  Serial.print(" B= ");
  Serial.println(blue);

 
  if (green < -20 && blue < -20 && red < -20) {
  
    Serial.println("Detected color: White");
    servoMotor.write(90); 
  } else {
    
    Serial.println("Detected color: Not white");
    servoMotor.write(120);
    delay(1800); 
    servoMotor.write(90); 
  }
}
