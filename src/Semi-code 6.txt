/* This code works with GY-31 TCS3200 TCS230 color sensor module
 *  It selects a photodiode set and reads its value (Red Set/Blue set/Green set) and displays it on the Serial monitor
 * and identify if possible the color
 * Refer to www.surtrtech.com for more details
 */
 
#define s0 9       // Module pins wiring
#define s1 10
#define s2 11
#define s3 12
#define out 13
#define ledPin 22   // LED control pin

int Red = 0, Blue = 0, Green = 0;  // RGB values 

void setup() 
{
   pinMode(s0, OUTPUT);    // Pin modes
   pinMode(s1, OUTPUT);
   pinMode(s2, OUTPUT);
   pinMode(s3, OUTPUT);
   pinMode(out, INPUT);
   pinMode(ledPin, OUTPUT); // LED control pin mode

   Serial.begin(9600);   // Initialize the serial monitor baud rate
   
   digitalWrite(s0, HIGH); // Putting S0/S1 on HIGH/HIGH levels means the output frequency scaling is at 100% (recommended)
   digitalWrite(s1, HIGH); // LOW/LOW is off, HIGH/LOW is 20%, and LOW/HIGH is 2%
   
   digitalWrite(ledPin, LOW); // Turn off the LED initially
}

void loop(){
  
  digitalWrite(ledPin, LOW); // Turn off the LED initially

  GetColors(); // Execute the GetColors function to get the value of each RGB color
               

   if (Red < Blue && Red <= Green && Red < 50) {
      Serial.println("Yellow"); // Red is identified when Red is the lowest and below a threshold
  }
  else if (Blue < Green && Blue < Red && Blue < 50) {
      Serial.println("Blue"); // Blue is identified when Blue is the lowest and below a threshold
  }
  else {
      Serial.println("White"); // If the color is not recognized
  }

  digitalWrite(ledPin, LOW); // Turn off the LED after taking a measurement

  delay(2000); // 2s delay, you can modify if you want
}

void GetColors()  
{    
  digitalWrite(s2, LOW); // S2/S3 levels define which set of photodiodes we are using
  digitalWrite(s3, LOW); // LOW/LOW is for RED, LOW/HIGH is for Blue, and HIGH/HIGH is for Green
  Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); // Measure Red
  delay(20);  
  digitalWrite(s3, HIGH); // Select the next color (set of photodiodes) and measure the value
  Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); // Measure Blue
  delay(20);  
  digitalWrite(s2, HIGH);  
  Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); // Measure Green
  delay(20);  
}

// Here's an example to understand the expression above,
// "digitalRead(out) == HIGH ? LOW : HIGH" this whole expression is either HIGH or LOW as pulseIn function requires a HIGH/LOW value on the second argument

// led_Status = led_Status == HIGH ? LOW : HIGH;
// if (led_Status == HIGH) 
// { 
//     led_Status = LOW; 
// } 
// else 
// { 
//     led_Status = HIGH; 
//}
