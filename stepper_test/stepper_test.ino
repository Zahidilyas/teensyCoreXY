#define DIR1 5                      //arduino ports
#define DIR2 6
#define STEP1 2
#define STEP2 3
#define ENABLE 8

void setup() {
  // put your setup code here, to run once:
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(STEP1,OUTPUT);
  pinMode(STEP2,OUTPUT);
  pinMode(ENABLE,OUTPUT);

  digitalWrite(ENABLE,LOW);

  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,LOW);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(STEP1,HIGH);
  digitalWrite(STEP2,HIGH);
  digitalWrite(STEP1,LOW);
  digitalWrite(STEP2,LOW);
  delayMicroseconds(100);
}
