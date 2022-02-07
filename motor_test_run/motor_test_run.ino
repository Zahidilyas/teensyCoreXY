const int stepPin = 2;
const int dirPin = 5;
int rot = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(stepPin,OUTPUT);
  pinMode(dirPin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 if(rot ==0){
    digitalWrite(dirPin, HIGH);
    rot = 1;
  }else {
    
    digitalWrite(dirPin, LOW);
    rot = 0;
  }

for(int x = 0; x<6600; x++) {
  digitalWrite(stepPin,HIGH);
  delayMicroseconds(100);
  digitalWrite(stepPin,LOW);
  delayMicroseconds(100);
}

delay(500);
}
