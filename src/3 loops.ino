
#include <SPI.h>
#include <Pixy2.h> // Include the correct library for PixyCam 2
#include <Servo.h>

// TCS3200 color sensor pins
#define S0 9
#define S1 10
#define S2 12
#define S3 11
#define sensorOut 13
#define OUTPUT_ENABLE_PIN 16 // Define the output enable pin



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

Servo servoMotor;

int myPins[6] = {2, 3, 4, 5, 6, 7}; // ENA, IN1, IN2, IN3, IN4, ENB
float deadZone = 0.15;
bool obstacleDetected = false;
int lastCubeColor = 0; // 1: Red, 2: Green

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
  // Enable the output before reading
  digitalWrite(OUTPUT_ENABLE_PIN, HIGH);
  delay(10); // Short delay to ensure the output is stable

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

  // Disable the output after reading
  digitalWrite(OUTPUT_ENABLE_PIN, LOW);

  // Print RGB values
  Serial.print("R= ");
  Serial.print(red);
  Serial.print(" G= ");
  Serial.print(green);
  Serial.print(" B= ");
  Serial.println(blue);
  

  
  if (green <= 0 && blue <= 0 && red <= 0) {
    // Detected white color
    moveRobot(125, 125);
    Serial.println("Detected color: White");
    servoMotor.write(90); // Keep servo at 90 degrees
  } else {
    // Detected any other color
    moveRobot(125, 125);
    Serial.println("Detected color: Not white");
    servoMotor.write(120); // Move servo to 120 degrees
    delay(1000); // Wait for 500 ms
    servoMotor.write(90); // Return servo to 90 degrees
  }
 
}



void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  
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
  
  // Set the output enable pin as output and initially enable it
  pinMode(OUTPUT_ENABLE_PIN, OUTPUT);
  digitalWrite(OUTPUT_ENABLE_PIN, HIGH); // Enable output by default

  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

}

void loop() {  
  checkColorSensor(); // Check the color sensor and adjust servo accordingl
}
