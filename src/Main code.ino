i#include <Servo.h>
#include <SPI.h>

// TCS3200 color sensor pins
#define S0 9
#define S1 10
#define S2 11
#define S3 12
#define sensorOut 13

// Ultrasonic sensor pins
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

//Pixy2 pixy;
Servo servoMotor;

// Motor control pins (ENA, IN1, IN2, IN3, IN4, ENB)
int motorPins[6] = {2, 3, 4, 5, 6, 7};
int c = 0;
float deadZone = 0.15;
bool obstacleDetected = false;
int lastCubeColor = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  // Attach servo motor to pin 8
  servoMotor.attach(8);

  // Set motor control pins as outputs
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
  }

  // Set color sensor pins as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
   float distance = getDistance();
  if (distance < 10.0) {
    // Turn left if obstacle detected
    Serial.println("Obstacle detected by ultrasonic sensor, turning left...");
    turnLeftWithServo();
    delay(500); // Adjust delay to control turn duration
  }

  // Check color sensor for line detection
  checkColorSensor();

 /* pixyCheck();
  Serial.println("ok");
   if (obstacleDetected) {
    if (lastCubeColor == 1) { // Red cube
      // Swerving right
      servoMotor.write(120); // Adjust servo position to turn right
      delay(500); // Adjust delay according to your control loop requirements for avoiding the obstacle

      // Continue moving forward and then correct to left
      while (pixyCheck() != 0); // Wait until the cube is not detected
      servoMotor.write(45); // Adjust servo position to turn lef
      delay(500); // Adjust delay according to your control loop requirements for correcting

      // Return to center
      servoMotor.write(90); // Return to center position
    } else if (lastCubeColor == 2) { // Green cube
      // Swerving left
      servoMotor.write(45); // Adjust servo position to turn left
      delay(500); // Adjust delay according to your control loop requirements for avoiding the obstacle

      //Continue moving forward and then correct to right
      while (pixyCheck() != 0); // Wait until the cube is not detected
      servoMotor.write(120); // Adjust servo position to turn right
      delay(500); // Adjust delay according to your control loop requirements for correcting

      // Return to center
      servoMotor.write(90); // Return to center position
    }

    obstacleDetected = false;
    lastCubeColor = 0; // Reset last cube color after avoiding the obstacle
    
  }*/

  
  // Check distance from ultrasonic sensor
 
}
/*float pixyCheck() {
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
}*/

float getDistance() {
  // Send a 10us pulse to the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in cm
  float distance = duration * 0.034 / 2;

  // Print the distance for debugging purposes
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

void checkColorSensor() {
  // Enable the output before reading
  delay(10); // Short delay to ensure the output is stable

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

  // Print RGB values
  Serial.print("R= ");
  Serial.print(red);
  Serial.print(" G= ");
  Serial.print(green);
  Serial.print(" B= ");
  Serial.println(blue);
  

  // Uncomment this section if you need to check for white color
  
  if (green <= 0 && blue <= 0 && red <= 0) {
    // Detected white color
    moveRobot(100, 100);
    Serial.println("Detected color: White");
    servoMotor.write(90); // Keep servo at 90 degrees
  } else {
    // Detected any other color
    moveRobot(100, 100);
    Serial.println("Detected color: Not white");
    servoMotor.write(120); // Move servo to 120 degrees
     moveRobot(100, 60);
    delay(920); // Wait for 1800 ms
    c++;
    servoMotor.write(90); // Return servo to 90 degrees
  }
   /*if (c == 12) {
    moveRobot(0, 0);
    delay(20000);
  }*/
  
}

void turnLeftWithServo() {
  // Turn left using the servo motor
  servoMotor.write(120); // Adjust servo position to turn left
  delay(500); // Adjust delay to control turn duration
  servoMotor.write(90); // Return servo to center position
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
