
#define ENCA 3
#define ENCB 4

int pos = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(pos);
  delay(10);
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b>0){
    pos++;
  }
  else{
    pos--;
  }
}
