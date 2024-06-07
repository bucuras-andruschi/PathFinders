
const int A1A = 3;
const int A2B = 4;
const int B1A = 5;
const int B2B = 6;

void setup() {
  pinMode(A1A,OUTPUT);
  pinMode(A2B,OUTPUT);
  pinMode(B1A,OUTPUT);
  pinMode(B2B,OUTPUT);

  // put your setup code here, to run once:

}

void loop() {
  digitalWrite(A1A,LOW);
  digitalWrite(A2B,HIGH);
  digitalWrite(B1A,LOW);
  digitalWrite(B2B,HIGH);
  delay(2000);
  // put your main code here, to run repeatedly:

}
