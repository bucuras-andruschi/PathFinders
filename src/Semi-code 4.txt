#include <Pixy2.h>

Pixy2 pixy;

int _output_PD5 = 5;
int _output_PD4 = 4;
int _output_PD7 = 7;
int _output_PD6 = 6;
int _output_PWM10 = 10;
int _output_PWM9 = 9;
int _led_L = 13;

const int detectionThreshold = 5;  
int redDetectionCount = 0;
int greenDetectionCount = 0;

void setup() {
    pinMode(_output_PD5, OUTPUT);
    pinMode(_output_PD4, OUTPUT);
    pinMode(_output_PD7, OUTPUT);
    pinMode(_output_PD6, OUTPUT);
    pinMode(_output_PWM10, OUTPUT);
    pinMode(_output_PWM9, OUTPUT);
    pinMode(_led_L, OUTPUT);

    Serial.begin(9600);
    pixy.init();
}

void inainte() {
    Serial.println("Moving forward");
    digitalWrite(_output_PD4, LOW);  
    digitalWrite(_output_PD5, HIGH);
    digitalWrite(_output_PD6, LOW); 
    digitalWrite(_output_PD7, HIGH);
    analogWrite(_output_PWM9, 100);  
    analogWrite(_output_PWM10, 100);
}

void stop() {
    Serial.println("Stopping");
    digitalWrite(_output_PD4, LOW);   
    digitalWrite(_output_PD5, LOW);
    digitalWrite(_output_PD6, LOW);   
    digitalWrite(_output_PD7, LOW);
    analogWrite(_output_PWM9, 0);     
    analogWrite(_output_PWM10, 0);
}

void turnLeft() {
    Serial.println("Turning left");
    digitalWrite(_output_PD4, HIGH); 
    digitalWrite(_output_PD5, LOW);
    digitalWrite(_output_PD6, LOW);   
    digitalWrite(_output_PD7, HIGH);
    analogWrite(_output_PWM9, 100);   
    analogWrite(_output_PWM10, 100);
}

void turnRight() {
    Serial.println("Turning right");
    digitalWrite(_output_PD4, LOW);   
    digitalWrite(_output_PD5, HIGH);
    digitalWrite(_output_PD6, HIGH);  
    digitalWrite(_output_PD7, LOW);
    analogWrite(_output_PWM9, 100);   
    analogWrite(_output_PWM10, 100);
}

void loop() {
    pixy.ccc.getBlocks();

    bool redDetected = false;
    bool greenDetected = false;

    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            if (pixy.ccc.blocks[i].m_signature == 1) {
                redDetected = true;
            } else if (pixy.ccc.blocks[i].m_signature == 2) {
                greenDetected = true;
            }
        }
    }

    if (redDetected) {
        redDetectionCount++;
        greenDetectionCount = 0;
    } else if (greenDetected) {
        greenDetectionCount++;
        redDetectionCount = 0;
    } else {
        redDetectionCount = 0;
        greenDetectionCount = 0;
    }

    if (redDetectionCount >= detectionThreshold) {
        turnLeft();
    } else if (greenDetectionCount >= detectionThreshold) {
        turnRight();
    } else {
        inainte();
    }

    delay(100); 
}
