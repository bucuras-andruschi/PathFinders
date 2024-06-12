
#define s0 9       
#define s1 10
#define s2 11
#define s3 12
#define out 13
#define ledPin 22  

int Red = 0, Blue = 0, Green = 0;  // RGB values 

void setup() 
{
   pinMode(s0, OUTPUT);   
   pinMode(s1, OUTPUT);
   pinMode(s2, OUTPUT);
   pinMode(s3, OUTPUT);
   pinMode(out, INPUT);
   pinMode(ledPin, OUTPUT); 

   Serial.begin(9600);   
   
   digitalWrite(s0, HIGH); 
   digitalWrite(s1, HIGH); 
   
   digitalWrite(ledPin, LOW); 
}

void loop(){
  
  digitalWrite(ledPin, LOW); 

  GetColors(); 
               

   if (Red < Blue && Red <= Green && Red < 50) {
      Serial.println("Yellow"); 
  }
  else if (Blue < Green && Blue < Red && Blue < 50) {
      Serial.println("Blue"); 
  }
  else {
      Serial.println("White"); 

  digitalWrite(ledPin, LOW); 

  delay(2000); 

void GetColors()  
{    
  digitalWrite(s2, LOW); 
  digitalWrite(s3, LOW); 
  Red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); 
  delay(20);  
  digitalWrite(s3, HIGH);
  Blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH); // Measure Blue
  delay(20);  
  digitalWrite(s2, HIGH);  
  Green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH)
  delay(20);  
}


