/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(28, 29);
Encoder knobRight(30, 31);
//   avoid using pins with LEDs attached

#define DIR1 5                      //arduino ports
#define DIR2 6
#define STEP1 2
#define STEP2 3

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(STEP1,OUTPUT);
  pinMode(STEP2,OUTPUT);
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,LOW);

  for(int i=0; i<3200;i++){
    digitalWrite(STEP1,HIGH);
  digitalWrite(STEP2,HIGH);
  digitalWrite(STEP1,LOW);
  digitalWrite(STEP2,LOW);
  delayMicroseconds(100);
  }
}

long positionLeft  = -999;
long positionRight = -999;

void loop() {
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}
