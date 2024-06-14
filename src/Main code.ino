#include <Servo.h>
#include <SPI.h>

#define S0 9
#define S1 10
#define S2 11
#define S3 12
#define sensorOut 13

#define TRIG_PIN 52
#define ECHO_PIN 53

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

Servo servoMotor;

int motorPins[6] = {2, 3, 4, 5, 6, 7};
int c = 0;
float deadZone = 0.15;
bool obstacleDetected = false;
int lastCubeColor = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  servoMotor.attach(8);
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  float distance = getDistance();
  if (distance < 10.0) {
    Serial.println("Obstacle detected by ultrasonic sensor, turning left...");
    turnLeftWithServo();
    delay(500);
  }
  checkColorSensor();
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void checkColorSensor() {
  delay(10);
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
  
  if (green <= 0 && blue <= 0 && red <= 0) {
    moveRobot(100, 100);
    Serial.println("Detected color: White");
    servoMotor.write(90);
  } else {
    moveRobot(100, 100);
    Serial.println("Detected color: Not white");
    servoMotor.write(120);
    moveRobot(100, 60);
    delay(920);
    c++;
    servoMotor.write(90);
  }
}

void turnLeftWithServo() {
  servoMotor.write(120);
  delay(500);
  servoMotor.write(90);
}

void moveRobot(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins[2], LOW);
  } else {
    digitalWrite(motorPins[1], LOW);
    digitalWrite(motorPins[2], HIGH);
  }

  if (rightSpeed >= 0) {
    digitalWrite(motorPins[3], HIGH);
    digitalWrite(motorPins[4], LOW);
  } else {
    digitalWrite(motorPins[3], LOW);
    digitalWrite(motorPins[4], HIGH);
  }

  analogWrite(motorPins[0], abs(leftSpeed));
  analogWrite(motorPins[5], abs(rightSpeed));
}
