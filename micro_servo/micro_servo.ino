/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo servo; 


void setup() {
  servo.attach(10);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  servo.write(0);
  delay(15);
}
