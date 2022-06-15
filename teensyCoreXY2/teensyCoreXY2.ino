//#include <Arduino.h>
#include "teensystep4.h"
#include <Servo.h>
#include "main.h"
#include <Encoder.h>
#include "../../../../../Desktop/presets.h"

// using namespace TS4; ?

// -------------------------------
// GLOBALS
// -------------------------------
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// ----- Bit set/clear/check/toggle macros
#define SET(x,y) (x |=(1<<y)) 
#define CLR(x,y) (x &= (~(1<<y)))
#define CHK(x,y) (x & (1<<y))
#define TOG(x,y) (x^=(1<<y))

// ----- motor definitions 
int ENCODER_PULSES_PER_REVOLUTION = 4000;
int STEPS_PER_REVOLITION  = 200*16;
#define STEPS_PER_MM STEPS_PER_REVOLITION/40      //200steps/rev; 16 x microstepping; 40mm/rev
#define NUDGE STEPS_PER_MM*5        //move pen 5mm (change number to suit)
#define DIR1 5                      //arduino ports
#define DIR2 6
#define STEP1 2
#define STEP2 3
#define ENABLE 8


#define NormalSwitchState 0       //This changes between previous core_xy and new_core_xy

// ----- Limit switches
const int x_switch = 7;
const int y_switch = 4;

bool CW = true;                     //flag ... does not affect motor direction
bool CCW = false;                   //flag ... does not affect motor direction
bool DIRECTION1;                    //motor directions can be changed in step_motors()
bool DIRECTION2;

long
PULSE_WIDTH = 100,                    //easydriver step pulse-width (uS)
DELAY = 200; //710                        //delay (uS) between motor steps (controls speed)


// ----- plotter definitions
#define BAUD 4800
#define XOFF 0x13                   //pause transmission (19 decimal)
#define XON 0x11                    //resume transmission (17 decimal)
#define PEN 9

float 
ARC_MAX = 2.0;                      //maximum arc-length (controls smoothness)

long
/*
   XY plotters only deal in integer steps.
*/
THIS_X = 0,                         //x co-ordinate (rounded)
THIS_Y = 0,                         //y co-ordinate (rounded)
LAST_X = 0,                         //x co-ordinate (rounded)
LAST_Y = 0;                         //y co-ordinate (rounded)

long
positionLeft = -999,
positionRight  = -999;

// ----- gcode definitions
#define STRING_SIZE 128             //string size

char
BUFFER[STRING_SIZE + 1],
       INPUT_CHAR;

String
INPUT_STRING,
SUB_STRING;

int
INDEX = 0,                        //buffer index
START,                            //used for sub_string extraction
FINISH,
Angle;

float X,Y,I,J;//gcode float values held here

// Create Instances of Classes
Servo servo_up_down;
Servo servo_rot;
Servo servo_claw;
TS4::Stepper stepperRight(STEP1,DIR1);
TS4::Stepper stepperLeft(STEP2,DIR2);
// ----- define and move a groups of steppers synchronized
TS4::StepperGroup stepperGroup{stepperRight, stepperLeft};

Encoder rightMotor(28,29);
Encoder leftMotor(30,31);

int current_servo_claw = 50;
int current_servo_zaxis = 50;
int current_servo_rot = 50;

// -----------------------
// SETUP
// -----------------------
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENABLE,OUTPUT);
  

  TS4::begin();

  stepperRight
      .setMaxSpeed(30'000) //10'000
      .setAcceleration(15'000); //15'000

  stepperLeft
      .setMaxSpeed(35'000)
      .setAcceleration(15'000);

  digitalWrite(DIR1, CW);
  digitalWrite(DIR2, CW);
  digitalWrite(STEP1, LOW);
  digitalWrite(STEP2, LOW);
  digitalWrite(ENABLE,LOW); // ----- Enable Stepper Motors

  // ----- Servos
  servo_claw.attach(8, SERVO_CLAW_LOW_PULSE_WIDTH , SERVO_CLAW_HIGH_PULSE_WIDTH);
  servo_up_down.attach(9, SERVO_Z_LOW_PULSE_WIDTH, SERVO_Z_HIGH_PULSE_WIDTH);
  servo_rot.attach(10,SERVO_ROT_LOW_PULSE_WIDTH , SERVO_ROT_HIGH_PULSE_WIDTH);
  gotoservo(90,"claw");
  gotoservo(0,"zaxis");
  gotoservo(0,"rot");

  // Set switched pins as Inputs
  pinMode(x_switch, INPUT);
  pinMode(y_switch, INPUT);

  // ----- plotter setup
  memset(BUFFER, '\0', sizeof(BUFFER));     //fill with string terminators
  INPUT_STRING.reserve(STRING_SIZE);
  INPUT_STRING = "";

  // ----- establish serial link
  Serial.begin(BAUD);

  // ----- flush the buffers
  Serial.flush();                           //clear TX buffer
  while (Serial.available()) Serial.read(); //clear RX buffer

  // ----- display commands
  menu();  
}

void gotoservo(int x,String motor){

  int current;
  if(motor == "claw"){
    current = current_servo_claw;
    if(x>current){
      for(int pos = current; pos <= x; pos += 1){ 
        servo_claw.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }else {
      for(int pos = current; pos >= x; pos -= 1){
        servo_claw.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }
    current_servo_claw = x;
    
  }else if(motor == "zaxis"){
    current = current_servo_zaxis;
    if(x>current){
      for(int pos = current; pos <= x; pos += 1){ 
        servo_up_down.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }else {
      for(int pos = current; pos >= x; pos -= 1){
        servo_up_down.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }
    current_servo_zaxis = x;
    
  }else if(motor == "rot"){
    current = current_servo_rot;
    if(x>current){
      for(int pos = current; pos <= x; pos += 1){ 
        servo_rot.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }else {
      for(int pos = current; pos >= x; pos -= 1){
        servo_rot.write(pos);              // tell servo to go to position in variable 'pos' 
        delay(15);                       // waits 15ms for the servo to reach the position 
      } 
    }
    current_servo_rot = x;
    
  }else{
    true;
  }
}

void loop()
{
  // ----- get the next instruction
  while (Serial.available()) {
    INPUT_CHAR = (char)Serial.read();         //read character
    Serial.write(INPUT_CHAR);                 //echo character to the screen
    BUFFER[INDEX++] = INPUT_CHAR;             //add char to buffer
    if (INPUT_CHAR == '\n') {                 //check for line feed
      Serial.flush();                         //clear TX buffer
      serialFlush();                          //clear RX buffer
      // Serial.write(XOFF);                     //pause transmission
      INPUT_STRING = BUFFER;                  //convert to string
      process(INPUT_STRING);                  //interpret string and perform task
      memset(BUFFER, '\0', sizeof(BUFFER));   //fill buffer with string terminators
      INDEX = 0;                              //point to buffer start
      INPUT_STRING = "";                      //empty string
      Serial.flush();                         //clear TX buffer
      // Serial.write(XON);                      //resume transmission
    }
  }
}

void serialFlush(){
  while(Serial.available() > 0) {
    Serial.read();
  }
}

//---------------------------------------------------------------------------
// MENU
//---------------------------------------------------------------------------
/*
   The Arduino F() flash macro is used to conserve RAM.
*/
void menu() {
  Serial.println(F(""));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("                         MENU"));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("    MENU ................ menu"));
  Serial.println(F("    G00 X## Y## ........ goto XY"));
  Serial.println(F("    G01 X## Y## ........ goto XY (arm-down)"));
  Serial.println(F("    T1 ................. Calibrate"));
  Serial.println(F("    T2 S##.## .......... set drawing Scale (1=100%)"));
  Serial.println(F("    T3 ................. Move up"));
  Serial.println(F("    T4 ................. Move down"));
  Serial.println(F("    Z1 ................. Grip Close"));
  Serial.println(F("    Z2 ................. Grip Open"));
  Serial.println(F("    R(angle) ........... Rotate with angle Ex. R90"));
  Serial.println(F("  ------------------------------------------------------"));
}

//--------------------------------------------------------------------------
// PROCESS
//--------------------------------------------------------------------------
void process(String string) {

  // ----- convert string to upper case
  INPUT_STRING = string;
  INPUT_STRING.toUpperCase();

  // ----------------------------------
  // G00   linear move with pen_up
  // ----------------------------------
  if (INPUT_STRING.startsWith("G00")) {
    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();
    }

//    pen_up();
    move_to(X, Y);
  }

  // ----------------------------------
  // G01   linear move with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G01")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();
    }

    pen_down();
    move_to(X, Y);
  }

  // ----------------------------------
  // G02   clockwise arc with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G02")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();
    }

    pen_down();
    draw_arc_cw(X, Y, I, J);
  }

  // ------------------------------------------
  // G03   counter-clockwise arc with pen_down
  // ------------------------------------------
  if (INPUT_STRING.startsWith("G03")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();
    }

    pen_down();
    draw_arc_ccw(X, Y, I, J);
  }

  // ----------------------------------
  // MENU
  // ----------------------------------
  if (INPUT_STRING.startsWith("MENU")) {
    menu();
  }

  // ----------------------------------
  // M1   manual position the arm over 0,0
  // ----------------------------------
  if (INPUT_STRING.startsWith("M1")) {

    // ----- variables
    int step;           //loop counter
    int steps = NUDGE;  //steps motor is to rotate

    // ----- instructions
    Serial.println(F(""));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    Position the end over the 0,0 co-ordinate:"));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    X-axis:             Y-axis:"));
    Serial.println(F("   'A'  'S'            'K'  'L'"));
    Serial.println(F("   <-    ->            <-    ->"));
    Serial.println(F("    Exit = 'E'"));

    // ----- flush the buffer
    while (Serial.available() > 0) Serial.read();

    // ----- control motors with 'A', 'S', 'K', and 'L' keys

    char keystroke = ' ';
    while (keystroke != 'E') {  //press 'E' key to exit

      // ----- check for keypress
      if (Serial.available() > 0) {
        keystroke = (char) Serial.read();
      }

      // ----- select task
      switch (keystroke) {
        case 'a':
        case 'A': {
            // ----- rotate motor1 CW
            Serial.println(steps);
            for (step = 0; step < steps; step++) {
              left();
            }
            keystroke = ' ';    //otherwise motor will continue to rotate
            break;
          }
        case 's':
        case 'S': {
            // ------ rotate motor1 CCW
            for (step = 0; step < steps; step++) {
              right();
            }
            keystroke = ' ';
            break;
          }
        case 'k':
        case 'K': {
            // ----- rotate motor2 CW
            for (step = 0; step < steps; step++) {
              up();
            }
            keystroke = ' ';
            break;
          }
        case 'l':
        case 'L': {
            // ----- rotate motor2 CCW
            for (step = 0; step < steps; step++) {
              down();
            }
            keystroke = ' ';
            break;
          }
        case 'q':
        case 'Q': {
            // ----- exit
            Serial.println(F(" "));
            Serial.println(F("  Quit ..."));
            keystroke = 'E';
            break;
          }
        case 'e':
        case 'E': {
            // ----- exit
            Serial.println(F(" "));
            Serial.println(F("  Calibration complete ..."));
            keystroke = 'E';

            // ----- initialise counters for co-ordinate (0,0)
            THIS_X = 0;                   //current X co-ordinate
            THIS_Y = 0;                   //current Y co-ordinate
            LAST_X = 0;                   //previous X co-ordinate
            LAST_Y = 0;                   //previous Y-co-ordinate
            break;
          }
        // ----- default for keystroke
        default: {
            break;
          }
      }
    }

    
  }


  if (INPUT_STRING.startsWith("DD")){
    for (int step = 0; step < NUDGE; step++) {
      right();
    }
  }
  if (INPUT_STRING.startsWith("LL")){
    for (int step = 0; step < NUDGE; step++) {
      left();
    }
  }
  if (INPUT_STRING.startsWith("FF")){
    for (int step = 0; step < NUDGE; step++) {
      up();
    }
  }
  if (INPUT_STRING.startsWith("BB")){
    for (int step = 0; step < NUDGE; step++) {
      down();
    }
  }

  
  // ----------------------------------
  // T3   pen up
  // ----------------------------------
  if (INPUT_STRING.startsWith("T3")) {
    pen_up();
  }

  // ----------------------------------
  // T4   pen down
  // ----------------------------------
  if (INPUT_STRING.startsWith("T4")) {
    pen_down();
  }

  // ----------------------------------
  // R   Rotate Arm
  // ----------------------------------
  if (INPUT_STRING.startsWith("R")) {
    
    // ----- extract Angle
    START = INPUT_STRING.indexOf('R');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Angle = SUB_STRING.toInt();
    }


    rotate(Angle);
//    servo_rot.write(Angle);
    delay(5); 
  }

  // ----------------------------------
  // T1   calibrate
  // ----------------------------------
  if (INPUT_STRING.startsWith("T1")) {
    calibrate();
//    servo_claw.write(180);
    stepperRight.setPosition(0);
    stepperLeft.setPosition(0);

    leftMotor.write(0);
    rightMotor.write(0);
    delay(200); 
  }
  
  // ----------------------------------
  // Z1   open claw
  // ----------------------------------
  if (INPUT_STRING.startsWith("Z1")) {
    grip_open();
//    servo_claw.write(180);
    delay(10); 
  }

  // ----------------------------------
  // Z2   close claw
  // ----------------------------------
  if (INPUT_STRING.startsWith("Z2")) {
    grip_close();
//    servo_claw.write(80);
    delay(10); 
  }
  
  if (INPUT_STRING.startsWith("O")) {
    
    // ----- extract Angle
    START = INPUT_STRING.indexOf('O');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Angle = SUB_STRING.toInt();
    }
    if(Angle>90){
        Angle=90;}
    if(Angle<40){
        Angle=40;}
    grip_open_close(Angle);
    delay(5); 
  }

  if (INPUT_STRING.startsWith("P")) {
    
    // ----- extract Angle
    START = INPUT_STRING.indexOf('P');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Angle = SUB_STRING.toInt();
    }
    if(Angle>180){
        Angle=180;}
    if(Angle<0){
        Angle=0;}
    gotoservo(Angle,"zaxis");
    delay(5); 
  }

}

// -------------------------------
// Calibrate
// -------------------------------
void calibrate(){
  // ----- variables
  int step;           //loop counter
  int steps = NUDGE;  //steps motor is to rotate

  //grip_open();
  // pen_down();
  
  while(digitalRead(x_switch)== NormalSwitchState){
    // x step
    for (step = 0; step < steps; step++) {
      right();
    }
  }
  
  while(digitalRead(y_switch)== NormalSwitchState){
    // y step
    for (step = 0; step < steps; step++) {
      down();
    }
  }

  LAST_X = 0;
  LAST_Y = 0;

  Serial.println(F("Calibrated"));
}


// -------------------------------
// MOVE_TO
// -------------------------------
void move_to(float x, float y) {

  long goalPosL, goalPosR;
  
  //x,y are absolute co-ordinates
 
  // ----- convert to steps
  THIS_X = round(x * STEPS_PER_MM);     
  THIS_Y = round(y * STEPS_PER_MM);

  int motorRightSteps;
  int motorLeftSteps;
  motorRightSteps = -1*THIS_X + -1*THIS_Y;
  motorLeftSteps  = -1*THIS_X + THIS_Y;

  goalPosL = int((float(motorLeftSteps)/float(STEPS_PER_REVOLITION))*float(ENCODER_PULSES_PER_REVOLUTION));
  goalPosR = int((float(motorRightSteps)/float(STEPS_PER_REVOLITION))*float(ENCODER_PULSES_PER_REVOLUTION));

  stepperRight.setTargetAbs(motorRightSteps);
  stepperLeft.setTargetAbs(motorLeftSteps);
  stepperGroup.move();
  delay(10);
  
  positionLeft = -1*leftMotor.read();
  positionRight = rightMotor.read();
  Serial.println(positionRight);
  Serial.println(goalPosR);
  Serial.println(positionLeft);
  Serial.println(goalPosL);


  int stepsToSetLeftMotor = int( (float(positionLeft)/float(ENCODER_PULSES_PER_REVOLUTION))*float(STEPS_PER_REVOLITION) );
  int stepsToSetRightMotor =int( (float(positionRight)/float(ENCODER_PULSES_PER_REVOLUTION))*float(STEPS_PER_REVOLITION) );

  if(abs(positionLeft-goalPosL) > 5 || abs(positionRight-goalPosR) > 5){
    
    stepperLeft.setPosition(stepsToSetLeftMotor);
    stepperRight.setPosition(stepsToSetRightMotor);
    
    for(int i = 0; i<4; i++){
      Serial.println(i);
      stepperRight.setTargetAbs(motorRightSteps);
      stepperLeft.setTargetAbs(motorLeftSteps);
      stepperGroup.move();
      delay(10);
      
      positionLeft = -1*leftMotor.read();
      positionRight = rightMotor.read();
      Serial.println(positionRight);
      Serial.println(goalPosR);
      Serial.println(positionLeft);
      Serial.println(goalPosL);

      if(abs(positionLeft-goalPosL) < 5 && abs(positionRight-goalPosR) < 5){
        break;
      }

      if(i==3){
        Serial.println("ERROR: COLLISION DETECTED");
      }

      stepsToSetLeftMotor = int( (float(positionLeft)/float(ENCODER_PULSES_PER_REVOLUTION))*float(STEPS_PER_REVOLITION) );
      stepsToSetRightMotor =int( (float(positionRight)/float(ENCODER_PULSES_PER_REVOLUTION))*float(STEPS_PER_REVOLITION) );
      stepperLeft.setPosition(stepsToSetLeftMotor);
      stepperRight.setPosition(stepsToSetRightMotor);
    }
    
  }
  
  // ----- remember last co-ordinate
  LAST_X = THIS_X;
  LAST_Y = THIS_Y;
}

//--------------------------------------------------------------------
// LEFT()           (move the pen 1 step left)
//--------- -----------------------------------------------------------
void left() {
  DIRECTION1 = CCW;
  DIRECTION2 = CCW;
  step_motors();
}

//--------------------------------------------------------------------
// RIGHT()           (move the pen 1 step right)
//--------- -----------------------------------------------------------
void right() {
  DIRECTION1 = CW;
  DIRECTION2 = CW;
  step_motors();
}

//--------------------------------------------------------------------
// UP()               (move the pen 1 step up)
//--------- -----------------------------------------------------------
void up() {
  DIRECTION1 = CCW;
  DIRECTION2 = CW;
  step_motors();
}

//--------------------------------------------------------------------
// DOWN()             (move the pen 1 step down)
//--------- -----------------------------------------------------------
void down() {
  DIRECTION1 = CW;
  DIRECTION2 = CCW;
  step_motors();
}

//----------------------------------------------------------------------------------------
// STEP MOTORS
//----------------------------------------------------------------------------------------
void step_motors() {
  digitalWriteFast(DIR1, DIRECTION1);
  digitalWriteFast(DIR2, DIRECTION2);

  digitalWriteFast(STEP1, LOW);
  digitalWriteFast(STEP2, LOW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWriteFast(STEP1, HIGH);
  digitalWriteFast(STEP2, HIGH);
  delayMicroseconds(PULSE_WIDTH);
}

//----------------------------------------------------------------------------
// DRAW ARC CLOCKWISE (G02)
//----------------------------------------------------------------------------
void draw_arc_cw(float x, float y, float i, float j) {

  // ----- inkscape sometimes produces some crazy values for i,j
  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
    move_to(x, y);
  } else {

    // ----- variables
    float
    thisX = LAST_X / (STEPS_PER_MM), //current  X co-ordinate
    thisY = LAST_Y / (STEPS_PER_MM), //current  Y co-ordinate
    nextX = x,                    //next X co-ordinate
    nextY = y,                    //next Y co-ordinate
    newX,                         //interpolated X co-ordinate
    newY,                         //interpolated Y co-ordinate
    I = i,                        //horizontal distance thisX from circle center
    J = j,                        //vertical distance thisY from circle center
    circleX = thisX + I,          //circle X co-ordinate
    circleY = thisY + J,          //circle Y co-ordinate
    delta_x,                      //horizontal distance between thisX and nextX
    delta_y,                      //vertical distance between thisY and nextY
    chord,                        //line_length between lastXY and nextXY
    radius,                       //circle radius
    alpha,                        //interior angle of arc
    beta,                         //fraction of alpha
    arc,                          //subtended by alpha
    current_angle,                //measured CCW from 3 o'clock
    next_angle;                   //measured CCW from 3 o'clock

    // ----- calculate arc
    delta_x = thisX - nextX;
    delta_y = thisY - nextY;
    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
    radius = sqrt(I * I + J * J);
    alpha = 2 * asin(chord / (2 * radius)); //see construction lines
    arc = alpha * radius;         //radians

    // ----- sub-divide alpha
    int segments = 1;
    if (arc > ARC_MAX) {
      segments = (int)(arc / ARC_MAX);
      beta = alpha / segments;
    } else {
      beta = alpha;
    }

    // ----- calculate current angle
    /*
      atan2() angles between 0 and PI are CCW +ve from 3 o'clock.
      atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
    */
    current_angle = atan2(-J, -I);
    if (current_angle <= 0) current_angle += 2 * PI;        //angles now 360..0 degrees CW

    // ----- plot intermediate CW co-ordinates
    next_angle = current_angle;                             //initialise angle
    for (int segment = 1; segment < segments; segment++) {
      next_angle -= beta;                                   //move CW around circle
      if (next_angle < 0) next_angle += 2 * PI;             //check if angle crosses zero
      newX = circleX + radius * cos(next_angle);            //standard circle formula
      newY = circleY + radius * sin(next_angle);
      move_to(newX, newY);
    }

    // ----- draw final line
    move_to(nextX, nextY);
  }
}

//----------------------------------------------------------------------------
// DRAW ARC COUNTER-CLOCKWISE (G03)
//----------------------------------------------------------------------------
/*
   We know the start and finish co-ordinates which allows us to calculate the
   chord length. We can also calculate the radius using the biarc I,J values.
   If we bisect the chord the center angle becomes 2*asin(chord/(2*radius)).
   The arc length may now be calculated using the formula arc_length = radius*angle.
*/
void draw_arc_ccw(float x, float y, float i, float j) {

  // ----- inkscape sometimes produces some crazy values for i,j
  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
    move_to(x, y);
  } else {

    // ----- variables
    float
    thisX = LAST_X ,  //current X co-ordinate
    thisY = LAST_Y ,  //current Y co-ordinate
    nextX = x,                      //next X co-ordinate
    nextY = y,                      //next Y co-ordinate
    newX,                           //interpolated X co-ordinate
    newY,                           //interpolated Y co-ordinate
    I = i,                          //horizontal distance thisX from circle center
    J = j,                          //vertical distance thisY from circle center
    circleX = thisX + I,            //circle X co-ordinate
    circleY = thisY + J,            //circle Y co-ordinate
    delta_x,                        //horizontal distance between thisX and nextX
    delta_y,                        //vertical distance between thisY and nextY
    chord,                          //line_length between lastXY and nextXY
    radius,                         //circle radius
    alpha,                          //interior angle of arc
    beta,                           //fraction of alpha
    arc,                            //subtended by alpha
    current_angle,                  //measured CCW from 3 o'clock
    next_angle;                     //measured CCW from 3 o'clock

    // ----- calculate arc
    delta_x = thisX - nextX;
    delta_y = thisY - nextY;
    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
    radius = sqrt(I * I + J * J);
    alpha = 2 * asin(chord / (2 * radius));     //see construction lines
    arc = alpha * radius;                       //radians

    // ----- sub-divide alpha
    int segments = 1;
    if (arc > ARC_MAX) {
      segments = (int)(arc / ARC_MAX);
      beta = alpha / segments;
    } else {
      beta = alpha;
    }

    // ----- calculate current angle
    /*
        tan2() angles between 0 and PI are CCW +ve from 3 o'clock.
        atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
    */
    current_angle = atan2(-J, -I);
    if (current_angle <= 0) current_angle += 2 * PI;        //angles now 360..0 degrees CW

    // ----- plot intermediate CCW co-ordinates
    next_angle = current_angle;                             //initialise angle
    for (int segment = 1; segment < segments; segment++) {
      next_angle += beta;                                   //move CCW around circle
      if (next_angle > 2 * PI) next_angle -= 2 * PI;        //check if angle crosses zero
      newX = circleX + radius * cos(next_angle);            //standard circle formula
      newY = circleY + radius * sin(next_angle);
      move_to(newX, newY);
    }

    // ----- draw final line
    move_to(nextX, nextY);
  }
}

//---------------------------------------------------------------------------
// PEN_UP / Down
// Raise the pen
// Changing the value in OCR2B changes the pulse-width to the SG-90 servo
//---------------------------------------------------------------------------
void pen_up() {
  //servo_up_down.write(10);
  gotoservo(0,"zaxis");
//  OCR1B = 148;                //1mS pulse
  delay(10);                 //give pen-lift time to respond
//  Serial.println("PEEEEEENN up");
}

void pen_down() {
  //servo_up_down.write(170);
  gotoservo(180,"zaxis");
  delay(10);                 //give pen-lift time to respond
}

//---------------------------------------------------------------------------
// Gripper Open/Close 
//---------------------------------------------------------------------------
void grip_open() {
  //servo_claw.write(20);
  gotoservo(90,"claw");
  delay(10);                 //give pen-lift time to respond
}

void grip_close() {
  //servo_claw.write(105);
  gotoservo(40,"claw");
  delay(10);                 //give pen-lift time to respond
}

void grip_open_close(int angle) {
  //servo_rot.write(angle);
  gotoservo(angle,"claw");
  //delay(10);                 //give pen-lift time to respond
}

//---------------------------------------------------------------------------
// Gripper Rotate 
//---------------------------------------------------------------------------
void rotate(int angle) {
  //servo_rot.write(angle);
  gotoservo(angle,"rot");
  //delay(10);                 //give pen-lift time to respond
}
