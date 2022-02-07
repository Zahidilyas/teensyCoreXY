#include <Servo.h>
Servo servo;

void setup() {
  servo.attach(11);
  
}
void loop() {
  int aa = 20;
  int bb = 105;
  servo.write(aa);
  delay(100);
  servo.write(aa);
  delay(100);
  servo.write(aa);
  delay(4000);
  servo.write(bb);
  delay(100);
  servo.write(bb);
  delay(100);
  servo.write(bb);
  delay(4000);
//  delay(10000);
//  servo.write(180);
//  delay(500);
//  delay(10000);
//  servo.write(90);
//  delay(500);
//  delay(10000);
//  servo.write(0);
//  delay(500);
//  delay(10000);
}
