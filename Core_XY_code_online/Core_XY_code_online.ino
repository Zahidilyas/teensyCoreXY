// -------------------------------
// GLOBALS
// -------------------------------

// ----- constants
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
#define STEPS_PER_MM 200*16/40      //200steps/rev; 16 x microstepping; 40mm/rev
#define NUDGE STEPS_PER_MM*5        //move pen 5mm (change number to suit)
#define DIR1 5                      //arduino ports
#define DIR2 6
#define STEP1 2
#define STEP2 3
#include <Servo.h>
Servo servo_up_down;
Servo servo_rot;
Servo servo_claw;

// Limit switches
const int x_switch = 7;
const int y_switch = 4;

bool CW = true;                     //flag ... does not affect motor direction
bool CCW = false;                   //flag ... does not affect motor direction
bool DIRECTION1;                    //motor directions can be changed in step_motors()
bool DIRECTION2;

long
PULSE_WIDTH = 20,                    //easydriver step pulse-width (uS)
DELAY = 200; //710                        //delay (uS) between motor steps (controls speed)

// ----- plotter definitions
#define BAUD 9600
#define XOFF 0x13                   //pause transmission (19 decimal)
#define XON 0x11                    //resume transmission (17 decimal)
#define PEN 9

float
SCALE_FACTOR = 2.0,                 //drawing scale (1 = 100%)
ARC_MAX = 2.0;                      //maximum arc-length (controls smoothness)

long
/*
   XY plotters only deal in integer steps.
*/
THIS_X = 0,                         //"scaled" x co-ordinate (rounded)
THIS_Y = 0,                         //"scaled" y co-ordinate (rounded)
LAST_X = 0,                         //"scaled" x co-ordinate (rounded)
LAST_Y = 0;                         //"scaled" y co-ordinate (rounded)

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

float
X,                                //gcode float values held here
Y,
I,
J;

// -----------------------
// SETUP
// -----------------------
void setup()
{
  // ----- initialise motor1
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  digitalWrite(DIR1, CW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP1, LOW); 

  // ----- initialise motor2
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  digitalWrite(DIR2, CW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP2, LOW);

  // ----- Servos
  servo_up_down.attach(9);
  servo_rot.attach(10);
  servo_claw.attach(11);

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

//--------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------
void loop() {

  // ----- get the next instruction
  while (Serial.available()) {
    INPUT_CHAR = (char)Serial.read();         //read character
    Serial.write(INPUT_CHAR);                 //echo character to the screen
    BUFFER[INDEX++] = INPUT_CHAR;             //add char to buffer
    if (INPUT_CHAR == '\n') {                 //check for line feed
      Serial.flush();                         //clear TX buffer
      Serial.write(XOFF);                     //pause transmission
      INPUT_STRING = BUFFER;                  //convert to string
      process(INPUT_STRING);                  //interpret string and perform task
      memset(BUFFER, '\0', sizeof(BUFFER));   //fill buffer with string terminators
      INDEX = 0;                              //point to buffer start
      INPUT_STRING = "";                      //empty string
      Serial.flush();                         //clear TX buffer
      Serial.write(XON);                      //resume transmission
    }
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
  Serial.println(F("    T1 ................. move to 0,0"));
  Serial.println(F("    T2 S##.## .......... set drawing Scale (1=100%)"));
  Serial.println(F("    T3 ................. Move up"));
  Serial.println(F("    T4 ................. Move down"));
  Serial.println(F("    Z1 ................. Grip Open"));
  Serial.println(F("    Z2 ................. Grip Close"));
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
        case 'e':
        case 'E': {
            // ----- exit
            Serial.println(F(" "));
            Serial.println(F("  Calibration complete ..."));
            keystroke = 'E';
            break;
          }
        // ----- default for keystroke
        default: {
            break;
          }
      }
    }

    // ----- initialise counters for co-ordinate (0,0)
    THIS_X = 0;                   //current X co-ordinate
    THIS_Y = 0;                   //current Y co-ordinate
    LAST_X = 0;                   //previous X co-ordinate
    LAST_Y = 0;                   //previous Y-co-ordinate
  }

  // ----------------------------------
  // T2   set scale factor
  // ----------------------------------
  if (INPUT_STRING.startsWith("T2")) {
    Serial.println("T2");

    START = INPUT_STRING.indexOf('S');
    if (!(START < 0)) {
      FINISH = START + 6;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH);
      SCALE_FACTOR = SUB_STRING.toFloat();
      Serial.print(F("Drawing now ")); Serial.print(SCALE_FACTOR * 100); Serial.println(F("%"));
    }
    else {
      Serial.println(F("Invalid scale factor ... try again. (1 = 100%)"));
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
    delay(2000); 
  }

  // ----------------------------------
  // T1   calibrate
  // ----------------------------------
  if (INPUT_STRING.startsWith("T1")) {
    calibrate();
//    servo_claw.write(180);
    delay(200); 
  }
  
  // ----------------------------------
  // Z1   open claw
  // ----------------------------------
  if (INPUT_STRING.startsWith("Z1")) {
    grip_open();
//    servo_claw.write(180);
    delay(200); 
  }

  // ----------------------------------
  // Z2   close claw
  // ----------------------------------
  if (INPUT_STRING.startsWith("Z2")) {
    grip_close();
//    servo_claw.write(80);
    delay(200); 
  }

}

// -------------------------------
// Calibrate
// -------------------------------
void calibrate(){
  // ----- variables
  int step;           //loop counter
  int steps = NUDGE;  //steps motor is to rotate

  grip_open();
  pen_down();
  
  while(digitalRead(x_switch)==1){
    // x step
    for (step = 0; step < steps; step++) {
      left();
//    Serial.println(digitalRead(x_switch));
//      Serial.println("Move right");
    }
  }
  
  while(digitalRead(y_switch)==1){
    // y step
    for (step = 0; step < steps; step++) {
//      Serial.println(digitalRead(y_switch));
//      Serial.println("Move right");
      down();
    }
  }
  
//  LAST_X = round(1086 * long(STEPS_PER_MM) * long(SCALE_FACTOR));
//  LAST_Y = round(657 * long(STEPS_PER_MM) * long(SCALE_FACTOR));
//  Serial.println(LAST_X);
//  Serial.println(LAST_Y);
  Serial.println("Calibrated");
}

// -------------------------------
// MOVE_TO
// -------------------------------
/*
   Assume that our sheet of paper has been
   "scaled" to match the stepping motors.
*/
void move_to(float x, float y) {                        //x,y are absolute co-ordinates

  // ----- apply scale factor
  THIS_X = round(x * STEPS_PER_MM * SCALE_FACTOR);      //scale x and y
  THIS_Y = round(y * STEPS_PER_MM * SCALE_FACTOR);

  // ----- draw a line between these "scaled" co-ordinates
  Serial.println(THIS_X);
  Serial.println(THIS_Y);
  Serial.println(LAST_X);
  Serial.println(LAST_Y);
  draw_line(LAST_X, LAST_Y, THIS_X, THIS_Y);

  // ----- remember last "scaled" co-ordinate
  LAST_X = THIS_X;
  LAST_Y = THIS_Y;
}

// ------------------------------------------------------------------------
// DRAW LINE
// ------------------------------------------------------------------------
/*
  This routine assumes that motor1 controls the x-axis and that motor2 controls
  the y-axis.

  The algorithm automatically maps all "octants" to "octant 0" and
  automatically swaps the XY coordinates if dY is greater than dX. A swap
  flag determines which motor moves for any combination X,Y inputs. The swap
  algorithm is further optimised by realising that dY is always positive
  in quadrants 0,1 and that dX is always positive in "quadrants" 0,3.

  Each intermediate XY co-ordinate is plotted which results in a straight line
*/
void draw_line(long x1, long y1, long x2, long y2) {          //these are "scaled" co-ordinates

  // ----- locals
  long
  x = x1,                               //current "scaled" X-axis position
  y = y1,                               //current "scaled" Y-axis position
  dy,                                   //line slope
  dx,
  slope,
  longest,                              //axis lengths
  shortest,
  maximum,
  error,                                //bresenham thresholds
  threshold;

  // ----- find longest and shortest axis
  dy = y2 - y1;                         //vertical distance
  dx = x2 - x1;                         //horizontal distance
  longest = max(abs(dy), abs(dx));      //longest axis
  shortest = min(abs(dy), abs(dx));     //shortest axis

  // ----- scale Bresenham values by 2*longest
  error = -longest;                     //add offset to so we can test at zero
  threshold = 0;                        //test now done at zero
  maximum = (longest << 1);             //multiply by two
  slope = (shortest << 1);              //multiply by two ... slope equals (shortest*2/longest*2)

  // ----- initialise the swap flag
  /*
     The XY axes are automatically swapped by using "longest" in
     the "for loop". XYswap is used to decode the motors.
  */
  bool XYswap = true;                    //used for motor decoding
  if (abs(dx) >= abs(dy)) XYswap = false;

  // ----- pretend we are always in octant 0
  /*
     The current X-axis and Y-axis positions will now be incremented (decremented) each time
     through the loop. These intermediate steps are parsed to the plot(x,y) function which calculates
     the number of steps required to reach each of these intermediate co-ordinates. This effectively
     linearises the plotter and eliminates unwanted curves.
  */
  for (int i = 0; i < longest; i++) {

    // ----- move left/right along X axis
    if (XYswap) {   //swap
      if (dy < 0) {
        y--;
        down();                         //move pen 1 step down
      } else {
        y++;
        up();                           //move pen 1 step up
      }
    } else {    //no swap
      if (dx < 0) {
        x--;
        left();                         //move pen 1 step left
      } else {
        x++;
        right();                        //move pen 1 step right
      }
    }

    // ----- move up/down Y axis
    error += slope;
    if (error > threshold) {
      error -= maximum;

      // ----- move up/down along Y axis
      if (XYswap) {  //swap
        if (dx < 0) {
          x--;
          left();                         //move pen 1 step left
        } else {
          x++;
          right();                        //move pen 1 step right
        }
      } else {  //no swap
        if (dy < 0) {
          y--;
          down();                         //move pen 1 step down
        } else {
          y++;
          up();                           //move pen 1 step up
        }
      }
    }
  }
}

//--------------------------------------------------------------------
// LEFT()           (move the pen 1 step left)
//--------- -----------------------------------------------------------
void right() {
  DIRECTION1 = CCW;
  DIRECTION2 = CCW;
  step_motors();
}

//--------------------------------------------------------------------
// RIGHT()           (move the pen 1 step right)
//--------- -----------------------------------------------------------
void left() {
  DIRECTION1 = CW;
  DIRECTION2 = CW;
  step_motors();
}

//--------------------------------------------------------------------
// UP()               (move the pen 1 step up)
//--------- -----------------------------------------------------------
void down() {
  DIRECTION1 = CW;
  DIRECTION2 = CCW;
  step_motors();
}

//--------------------------------------------------------------------
// DOWN()             (move the pen 1 step down)
//--------- -----------------------------------------------------------
void up() {
  DIRECTION1 = CCW;
  DIRECTION2 = CW;
  step_motors();
}

//----------------------------------------------------------------------------------------
// STEP MOTORS
//----------------------------------------------------------------------------------------
void step_motors() {

  // ----- locals
//  enum {dir1, step1, dir2, step2};  //define bit positions
  byte dir1 = B00100000;
  byte dir2 = B01000000;
  byte step1 = B00000100;
  byte step2 = B00001000;
  byte pattern = PORTD;                                           //read current state PORTB

  // ----- set motor directions
  //(DIRECTION1 == CW) ? SET(pattern, dir1) : CLR(pattern, dir1); //normal motor direction
  //(DIRECTION2 == CW) ? SET(pattern, dir2) : CLR(pattern, dir2); //normal motor direction
  if(DIRECTION1 == CW) {pattern = pattern | dir1 ;} else {pattern = pattern & ~dir1;};  //motor windings reversed
  if(DIRECTION2 == CW) {pattern = pattern | dir2 ;} else {pattern = pattern & ~dir2;};  
  PORTD = pattern;
  delayMicroseconds(PULSE_WIDTH);                                 //wait for direction lines to stabilise

  // ----- create leading edge of step pulse(s)
  pattern = pattern | step1;                                  //prepare step pulse
  pattern = pattern | step2;
  PORTD = pattern;                                                //step the motors
  delayMicroseconds(PULSE_WIDTH);                                 //mandatory delay

  // ----- create trailing-edge of step-pulse(s)
  pattern = pattern & ~step1;
  pattern = pattern & ~step2;
  PORTD = pattern;

  // ----- determines plotting speed
  delayMicroseconds(DELAY);   
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
    thisX = LAST_X / (STEPS_PER_MM * SCALE_FACTOR), //current unscaled X co-ordinate
    thisY = LAST_Y / (STEPS_PER_MM * SCALE_FACTOR), //current unscaled Y co-ordinate
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
    thisX = LAST_X / SCALE_FACTOR,  //current unscaled X co-ordinate
    thisY = LAST_Y / SCALE_FACTOR,  //current unscaled Y co-ordinate
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
  servo_up_down.write(10);
//  OCR1B = 148;                //1mS pulse
  delay(250);                 //give pen-lift time to respond
//  Serial.println("PEEEEEENN up");
}

void pen_down() {
  servo_up_down.write(170);
  delay(250);                 //give pen-lift time to respond
}

//---------------------------------------------------------------------------
// Gripper Open/Close 
//---------------------------------------------------------------------------
void grip_open() {
  servo_claw.write(20);
  delay(250);                 //give pen-lift time to respond
}

void grip_close() {
  servo_claw.write(105);
  delay(250);                 //give pen-lift time to respond
}

//---------------------------------------------------------------------------
// Gripper Rotate 
//---------------------------------------------------------------------------
void rotate(int angle) {
  servo_rot.write(angle);
  delay(250);                 //give pen-lift time to respond
}
