#include <Stepper.h>

//Initialize the stepper library on pins 2 and 5

// Stepper Motor X
const int stepPin = 2; //X.STEP
const int dirPin = 5; // X.DIR
const int stepsPerRevolution = 200;
int rpm = 400; // max = 480
byte direction = 1; // 0 for cw and 1 for ACW

long steptime = 300000/rpm; // rpm = a/360 * 1/steptime * 60
int steptime_ms = 0;
 
void setup() {
// Sets the two pins as Outputs
pinMode(stepPin,OUTPUT); 
pinMode(dirPin,OUTPUT);

digitalWrite(dirPin,direction); // Enables the motor to move in a particular direction

Serial.begin(9600);
}


void loop() {
// Makes 200 pulses for making one full cycle rotation

if (Serial.available() > 0) {
  rpm = Serial.read() - '0';
  steptime = 300000/rpm;
  delayMicroseconds(1000);
}
if (rpm!=0){
  for(int x = 0; x < 800; x++) {
    if (Serial.available() > 0) {
      rpm = Serial.read() - '0';
      steptime = 300000/rpm;
      delayMicroseconds(1000);
    }
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(20); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(20); 
  
    if ((steptime-40)<=10000){
      delayMicroseconds(steptime - 40);
    }else {
      steptime_ms = steptime/1000;
      delay(steptime_ms);
    } 
  }
}
delay(1000); // One second delay
//Serial.println(rpm);

}
